# Random Pick (OpenVINO Grasp Detection)

A simple application demonstrating how to pick up objects from clutter scenarios with an industrial robot arm.
The application takes grasp detection results from [OpenVINO GPD](https://github.com/atenpas/gpd/blob/master/tutorials/tutorial_openvino.md),
transforms the grasp pose from camera view
to the robot view with the [Hand-Eye Calibration](https://github.com/crigroup/handeye),
translates the [Grasp Pose](https://github.com/atenpas/gpd/blob/master/msg/GraspConfig.msg) into [moveit_msgs Grasp](http://docs.ros.org/api/moveit_msgs/html/msg/Grasp.html),
and uses the [MoveGroupInterface](https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html) to pick and place the object.
Watch this [demo_video](https://www.youtube.com/embed/b4EPvHdidOA?rel=0) to see the output of this application.
