#! /usr/bin/env python

import rospy
import IPython
import argparse
import actionlib
import control_msgs.msg
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO, SetIORequest, SetIOResponse

def parse_args():
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='Default info')
  parser.add_argument('--IPython', action="store_true",
              help='If set, will embed IPython for deeper debugging')
  parser.add_argument('--sim', action="store_true",
              help='If set, will use simulation mode')
  args = parser.parse_args(rospy.myargv()[1:])
  return args

class HitbotAction(object):
  _feedback = control_msgs.msg.GripperCommandFeedback()
  _result = control_msgs.msg.GripperCommandResult()

  def __init__(self, action_name = "/gripper_cmd", service_name = "/ur_driver/set_io", sim = False):
    self._action_name = action_name
    self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, 
                                                               execute_cb=self.execute_cb, 
                                                               auto_start=False)
    self._sim = sim
    self._rate = rospy.Rate(1)
    self._open = False
    self._pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

    if self._sim:
      self._as.start()
      self._init = True
    else:
      try:
        rospy.wait_for_service(service_name, timeout=3)
        self._set_IO_srv = rospy.ServiceProxy(service_name, SetIO)
        self._as.start()
        self._init = True
        rospy.loginfo("Hitbot server initialized successfully.")
      except rospy.ServiceException, e:
        self._init = False
        rospy.logerr("Call service {} failed: {}".format(service_name, e))
      except rospy.exceptions.ROSException, e:
        self._init = False
        rospy.logerr("Call service {} failed: {}".format(service_name, e))

    if self._init:
      if not (self.powerup_gripper and self.close_gripper() and self.open_gripper()):
        exit(-1)
    else:
      exit(-1)

  def start(self):
    """
    start the control loop
    """
    r = rospy.Rate(50)
    while not rospy.is_shutdown():
      js = JointState()
      js.header.stamp = rospy.Time.now()
      js.name += ["hitbot_base_finger0_joint"]
      js.name += ["hitbot_base_finger1_joint"]
      js.effort += [0.0, 0.0]
      js.velocity += [0.0, 0.0]
      if self._open:
        js.position += [-0.01, 0.01]
      else:
        js.position += [0.0, 0.0]
      self._pub.publish(js)
      r.sleep()
    exit(-1)

  def execute_cb(self, goal):
    """
    Gripper action server callback
    """
    if self._init:
      if goal.command.position != 0:
        if self.open_gripper():
          self._result.reached_goal = True
          self._as.set_succeeded(self._result)
        else:
          self._result.reached_goal = False
          self._as.set_aborted(self._result)
      else:
        if self.close_gripper():
          self._result.reached_goal = True
          self._as.set_succeeded(self._result)
        else:
          self._result.reached_goal = False
          self._as.set_aborted(self._result)
    else:
      rospy.logerr("Hitbot action server not initialized.")

  def powerup_gripper(self):
    if self._sim:
      rospy.loginfo("Set the gripper voltage in simulation.")
      return True
    try:
      # open gripper
      req = SetIORequest()
      # set tool0 true
      req.fun = 4
      req.state = 24
      self._set_IO_srv(req)
      rospy.loginfo("Set the gripper voltage.")
      self._rate.sleep()
      return True
    except rospy.ServiceException, e:
      rospy.logerr("Failed to set gripper voltage: {}.".format(e))
      return False

  def open_gripper(self):
    if self._sim:
      rospy.loginfo("Open gripper in simulation.")
      self._open = True
      return True
    try:
      # open gripper
      req = SetIORequest()
      # set tool0 true
      req.fun = 1
      req.pin = 16
      req.state = 1.0
      self._set_IO_srv(req)
      # set tool1 false
      req.fun = 1
      req.pin = 17
      req.state = 0.0
      self._set_IO_srv(req)
      rospy.loginfo("Open gripper.")
      self._rate.sleep()
      self._open = True
      return True
    except rospy.ServiceException, e:
      rospy.logerr("Failed to open hitbot gripper: {}.".format(e))
      return False

  def close_gripper(self):
    if self._sim:
      self._open = False
      rospy.loginfo("Close gripper in simulation.")
      return True
    try:
      # close gripper
      req = SetIORequest()
      # set tool0 false
      req.fun = 1
      req.pin = 16
      req.state = 0.0
      self._set_IO_srv(req)
      # set tool1 true
      req.fun = 1
      req.pin = 17
      req.state = 1.0
      self._set_IO_srv(req)
      rospy.loginfo("Close gripper")
      self._rate.sleep()
      self._open = False
      return True
    except rospy.ServiceException, e:
      rospy.logerr("Failed to close hitbot gripper: {}".format(e))
      return False

if __name__ == "__main__":
  args = parse_args()
  rospy.init_node("Hitbot_driver")
  server = HitbotAction(action_name="gripper_cmd", service_name="/ur_driver/set_io", sim=args.sim)
  server.start()
