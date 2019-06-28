/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Intel Corporation.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sharron Liu */

#include <mutex>
// GPD
#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>
// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// ROS
#include <ros/ros.h>

// TF2
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Boost
#include <boost/program_options.hpp>

static std::mutex m_;
static bool isGrasping = false;

static const char target_frame_id[] = "base";
/* place position in the target_frame_id*/
static std::vector<double> place_position = {-0.45, -0.30, 0.25};
/* place position in joint values*/
static std::vector<double> joint_values_place = {0.3857,  -1.3737, 1.8270,
                                                 -2.0249, -1.5708, 1.9569};
/* pre-pick position in joint values*/
static std::vector<double> joint_values_pick = {1.3067,  -1.3066, 1.4736,
                                                -1.7380, -1.5708, 0};

/* gripper parameters: */
static std::vector<std::string> finger_joint_names = {
    "hitbot_base_finger0_joint", "hitbot_base_finger1_joint"};
static std::vector<double> finger_positions_open = {-0.01, 0.01};
static std::vector<double> finger_positions_close = {0.0, 0.0};
static double approach_distance = 0.1;
/* end-effector yaw offset*/
static double eef_yaw_offset = M_PI / 4;
/* offset from the gripper base (finger root) to the parent link of eef (end of
 * robot arm)*/
static double eef_offset = 0.154;

/* work table parameters: */
/* workspace boundy, described as a cube {x_min, x_max, y_min, y_max, z_min,
 * z_max}
 * in metres in the target_frame_id*/
static std::vector<double> boundry = {-0.3, 0.1, -0.7, -0.35, -0.15, 0.05};
/* minimum height in metres (altitude above the work table) of object to grasp*/
static double object_height_min = 0.028;

/* grasp parameters*/
/* minimum score*/
static const float kThresholdScore = 1;
/* expected grasp approach direction*/
static tf2::Vector3 grasp_approach(0, 0, -1);
/* maximum approach deviation*/
static double approach_deviation = M_PI / 9;
/* grasp position offset introduced by the system (e.g. camera, hand-eye
 * calibration, etc.)
 * {x_offset, y_offset} in metres in the target_frame_id*/
static std::vector<double> grasp_position_offset = {0.006, 0.003};
/* grasp candidates*/
static std::vector<std::pair<moveit_msgs::Grasp, geometry_msgs::Point>>
    grasp_candidates;

void openGripper(trajectory_msgs::JointTrajectory &posture) {
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = finger_joint_names[0];
  posture.joint_names[1] = finger_joint_names[1];

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = finger_positions_open[0];
  posture.points[0].positions[1] = finger_positions_open[1];
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture) {
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = finger_joint_names[0];
  posture.joint_names[1] = finger_joint_names[1];

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = finger_positions_close[0];
  posture.points[0].positions[1] = finger_positions_close[1];
  posture.points[0].time_from_start = ros::Duration(0.5);
}

moveit::planning_interface::MoveItErrorCode
pick(moveit::planning_interface::MoveGroupInterface &move_group,
     moveit_msgs::Grasp &grasp) {
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.push_back(grasp);

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  // Call pick to pick up the object using the grasps given
  move_group.setStartStateToCurrentState();
  move_group.setPoseReferenceFrame(grasp.grasp_pose.header.frame_id);
  return move_group.pick("object", grasps);
}

moveit::planning_interface::MoveItErrorCode
place(moveit::planning_interface::MoveGroupInterface &move_group) {
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  place_location[0].place_pose.header.frame_id = target_frame_id;
  tf2::Quaternion quat;
  quat.setRPY(M_PI, 0, 0); // base to tool0
  place_location[0].place_pose.pose.orientation = tf2::toMsg(quat);
  place_location[0].place_pose.pose.position.x = place_position[0];
  place_location[0].place_pose.pose.position.y = place_position[1];
  place_location[0].place_pose.pose.position.z = 0.0;

  // Setting pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id =
      target_frame_id;
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = approach_distance / 2;
  place_location[0].pre_place_approach.desired_distance = approach_distance;

  // Setting post-grasp retreat
  place_location[0].post_place_retreat.direction.header.frame_id =
      target_frame_id;
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = approach_distance / 2;
  place_location[0].post_place_retreat.desired_distance = approach_distance;

  openGripper(place_location[0].post_place_posture);

  // Call place to place the object using the place locations given.
  move_group.setStartStateToCurrentState();
  move_group.setPoseReferenceFrame(target_frame_id);
  return move_group.place("object", place_location);
}

void reset_pose(moveit::planning_interface::MoveGroupInterface &move_group) {
  move_group.setStartStateToCurrentState();
  move_group.setPoseReferenceFrame(target_frame_id);

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = place_position[0];
  target_pose1.position.y = place_position[1];
  target_pose1.position.z = place_position[2];

  tf2::Quaternion orientation;
  orientation.setRPY(M_PI, 0, 0);
  target_pose1.orientation = tf2::toMsg(orientation);

  move_group.setPoseTarget(target_pose1, move_group.getEndEffectorLink());

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("plan pose goal %s", success ? "SUCCESS" : "FAILED");

  success = (move_group.execute(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("execute pose goal %s", success ? "SUCCESS" : "FAILED");
}

void reset_joint(moveit::planning_interface::MoveGroupInterface &move_group,
                 const std::vector<double> &joint_values) {
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("plan joint_value goal %s", success ? "SUCCESS" : "FAILED");

  success = (move_group.execute(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("execute joint_value goal %s", success ? "SUCCESS" : "FAILED");
}

// check boundry
bool check_boundry(double x, double y, double z) {
  ROS_DEBUG("base point [%f %f %f]", x, y, z);
  return (x >= boundry[0] && x <= boundry[1] && y >= boundry[2] &&
          y <= boundry[3] && z >= boundry[4] && z <= boundry[5]);
}

moveit_msgs::Grasp to_moveit(gpd::GraspConfig &grasp,
                             const std_msgs::Header &header) {
  moveit_msgs::Grasp msg;
  msg.grasp_pose.header = header;
  msg.grasp_quality = grasp.score.data;

  double offset = eef_offset;
  /* make sure a distance of 'object_height_min/2' from tabletop to fingertip.*/
  offset += (boundry[4] + object_height_min / 2) - grasp.top.z;
  ROS_DEBUG("offset is %f", offset);

  // set grasp position, translation from hand-base to the parent-link of EEF
  msg.grasp_pose.pose.position.x = grasp.bottom.x - grasp.approach.x * offset;
  msg.grasp_pose.pose.position.y = grasp.bottom.y - grasp.approach.y * offset;
  msg.grasp_pose.pose.position.z = grasp.bottom.z - grasp.approach.z * offset;

  // rotation matrix
  // https://github.com/atenpas/gpd/blob/master/tutorials/hand_frame.png
  tf2::Matrix3x3 r(grasp.binormal.x, grasp.axis.x, grasp.approach.x,
                   grasp.binormal.y, grasp.axis.y, grasp.approach.y,
                   grasp.binormal.z, grasp.axis.z, grasp.approach.z);
  tf2::Quaternion quat;
  r.getRotation(quat);
  // EEF yaw-offset to its parent-link (last link of arm)
  quat *= tf2::Quaternion(tf2::Vector3(0, 0, 1), eef_yaw_offset);
  quat.normalize();
  // set grasp orientation
  msg.grasp_pose.pose.orientation = tf2::toMsg(quat);
  ROS_INFO_STREAM("*** MoveIt pick pose/tool0 " << msg.grasp_pose);

  // set pre-grasp approach
  msg.pre_grasp_approach.direction.header = header;
  msg.pre_grasp_approach.direction.vector = grasp.approach;
  msg.pre_grasp_approach.min_distance = approach_distance / 2;
  msg.pre_grasp_approach.desired_distance = approach_distance;

  // set post-grasp retreat
  msg.post_grasp_retreat.direction.header = header;
  msg.post_grasp_retreat.direction.vector.x = -grasp.approach.x;
  msg.post_grasp_retreat.direction.vector.y = -grasp.approach.y;
  msg.post_grasp_retreat.direction.vector.z = -grasp.approach.z;
  msg.post_grasp_retreat.min_distance = approach_distance / 2;
  msg.post_grasp_retreat.desired_distance = approach_distance;

  return msg;
}

void gpd_cb(const gpd::GraspConfigList::ConstPtr &msg) {
  {
    std::lock_guard<std::mutex> lock(m_);
    if (isGrasping) {
      ROS_DEBUG("===========grasping, skip detection");
      return;
    } else {
      ROS_DEBUG("===========not grasping, do detection");
    }
  }

  static tf::TransformListener listener(ros::Duration(10));
  geometry_msgs::PointStamped from_bottom, to_bottom, from_top, to_top,
      from_surface, to_surface;
  geometry_msgs::Vector3Stamped from_approach, to_approach, from_binormal,
      to_binormal, from_axis, to_axis;
  gpd::GraspConfig to_grasp;
  std_msgs::Header header;
  for (auto grasp : msg->grasps) {
    if (grasp.score.data < kThresholdScore) {
      ROS_INFO("skipping low score grasp %f", grasp.score.data);
      continue;
    }
    from_bottom.point = grasp.bottom;
    from_bottom.header = msg->header;
    from_top.point = grasp.top;
    from_top.header = msg->header;
    from_surface.point = grasp.surface;
    from_surface.header = msg->header;
    from_approach.vector = grasp.approach;
    from_approach.header = msg->header;
    from_binormal.vector = grasp.binormal;
    from_binormal.header = msg->header;
    from_axis.vector = grasp.axis;
    from_axis.header = msg->header;
    try {
      listener.transformPoint(target_frame_id, from_bottom, to_bottom);
      listener.transformPoint(target_frame_id, from_top, to_top);
      listener.transformPoint(target_frame_id, from_surface, to_surface);
      listener.transformVector(target_frame_id, from_approach, to_approach);
      listener.transformVector(target_frame_id, from_binormal, to_binormal);
      listener.transformVector(target_frame_id, from_axis, to_axis);
    } catch (tf::TransformException &ex) {
      ROS_DEBUG("skipping transformation failed");
      continue;
    }
    // error measured after hand-eye calibration
    to_bottom.point.x += grasp_position_offset[0];
    to_bottom.point.y += grasp_position_offset[1];
    to_top.point.x += grasp_position_offset[0];
    to_top.point.y += grasp_position_offset[1];
    to_surface.point.x += grasp_position_offset[0];
    to_surface.point.y += grasp_position_offset[1];

    // skip deviation grasp poses
    tf2::Vector3 approach(to_approach.vector.x, to_approach.vector.y,
                          to_approach.vector.z);
    double ang = tf2::tf2Angle(grasp_approach, approach);
    if (std::isnan(ang) || ang < -approach_deviation ||
        ang > approach_deviation) {
      ROS_INFO("skipping deviation approach");
      continue;
    }

    if (check_boundry(to_bottom.point.x, to_bottom.point.y,
                      to_bottom.point.z)) {
      ROS_INFO("*** Grasp position-top (%f, %f, %f) score [%f]", to_top.point.x,
               to_top.point.y, to_top.point.z, grasp.score.data);
      ROS_DEBUG("approach angle %f", ang);
      to_grasp.bottom = to_bottom.point;
      to_grasp.top = to_top.point;
      to_grasp.surface = to_surface.point;
      to_grasp.approach = to_approach.vector;
      to_grasp.binormal = to_binormal.vector;
      to_grasp.axis = to_axis.vector;

      header.stamp = msg->header.stamp;
      header.frame_id = target_frame_id;

      std::lock_guard<std::mutex> lock(m_);
      grasp_candidates.clear();
      grasp_candidates.push_back(
          std::make_pair(to_moveit(to_grasp, header), to_surface.point));
      break;
    } else {
      ROS_INFO("skipping out of boundary");
    }
  }
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface
                             &planning_scene_interface,
                         geometry_msgs::Pose &pose) {
  // Create vector to hold 1 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Define the object that we will be manipulating
  collision_objects[0].header.frame_id = target_frame_id;
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type =
      collision_objects[1].primitives[0].SPHERE;
  collision_objects[0].primitives[0].dimensions.resize(1);
  collision_objects[0].primitives[0].dimensions[0] = 0.01;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0] = pose;

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv) {
  // Define and parse the program options
  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()("help,h", "Random Pick help.")(
      "runtest,r", po::value<bool>()->default_value(false),
      "Flag to exit when pick and place succeed once.");
  po::variables_map vm;
  bool runtest = false;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("runtest")) {
      runtest = vm["runtest"].as<bool>();
      std::cout << "Make runtest exit: " << runtest << std::endl;
    }
  } catch (po::error &e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return -1;
  }

  // Start the program
  ros::init(argc, argv, "random_pick");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::MoveGroupInterface group("ur5_arm");
  group.setPlanningTime(45.0);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::WallDuration(1.0).sleep();
  reset_joint(group, joint_values_place);

  geometry_msgs::PoseStamped pose = group.getCurrentPose("tool0");
  std::vector<double> rpy = group.getCurrentRPY("tool0");
  ROS_INFO_STREAM("*********** tool0 pose " << pose);
  ROS_INFO_STREAM("*********** tool0 rpy " << rpy[0] << ", " << rpy[1] << ", "
                                           << rpy[2]);

  ros::WallDuration(1.0).sleep();
  ros::Subscriber sub =
      nh.subscribe("/detect_grasps/clustered_grasps", 1, gpd_cb);

  moveit_msgs::Grasp grasp;
  gpd::GraspConfig gpd_grasp;
  geometry_msgs::Pose obj_pose;
  while (ros::ok()) {
    while (grasp_candidates.empty()) {
      ros::WallDuration(0.1).sleep();
    }
    {
      std::lock_guard<std::mutex> lock(m_);
      if (!grasp_candidates.empty()) {
        isGrasping = true;
        ROS_INFO("===========do grasping");
        grasp = grasp_candidates.back().first;
        obj_pose.position = grasp_candidates.back().second;
        grasp_candidates.pop_back();
      } else {
        continue;
      }
    }
    // add object to scene
    obj_pose.orientation = grasp.grasp_pose.pose.orientation;
    addCollisionObjects(planning_scene_interface, obj_pose);

    // move to reset_joint for pick
    reset_joint(group, joint_values_pick);
    // pick
    moveit::planning_interface::MoveItErrorCode ret = pick(group, grasp);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      ROS_WARN_STREAM("PICK RETURN code " << ret);
      continue;
    }

    // move to reset_joint for place
    reset_joint(group, joint_values_place);
    {
      std::lock_guard<std::mutex> lock(m_);
      isGrasping = false;
    }
    // place
    ret = place(group);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      ROS_WARN_STREAM("PLACE RETURN code " << ret);
      continue;
    }

    // remove object from scene
    std::vector<std::string> objs;
    objs.push_back("object");
    planning_scene_interface.removeCollisionObjects(objs);

    if (runtest)
      return moveit::planning_interface::MoveItErrorCode::SUCCESS;
  }

  ros::waitForShutdown();
  return 0;
}
