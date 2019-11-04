Bring up fake controller, real controller and gripper controller
================================================================

This application runs with either a fake controller **OR** a real controller.

.. attention:: It's very important before running the application on a real robot,
               you should verify your platform setup with the "fake controller".

**Option 1**: launch a fake controller for UR5

.. code-block:: bash

  roslaunch ur5_hitbot_ilc_platform_moveit_config demo.launch

**Option 2**: Launch a real contoller for UR5

.. code-block:: bash

  roslaunch random_pick ur5_bringup.launch robot_ip:='ip address of the robot controller'

  roslaunch random_pick hitbot.launch # Optional if a hitbot gripper is used

  roslaunch random_pick ur5_moveit_planning_execution.launch # start motion planning pipeline
