Setup UR5 With MoveIt
=====================

This section contains the instructions to install the dependencies
that are required to make UR5 robot run in simulation and real execution with MoveIt.

System Requirement
-------------------
Ubuntu Linux 18.04 on 64-bit

Dependencies
------------

Install `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_.

Install `MoveIt <https://moveit.ros.org/install/source/>`_.

Install `ros_control <http://wiki.ros.org/ros_control>`_: ::

  sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers

Download and install ROS-Industrial_, universal_robot_ and  ur_modern_driver_.
universal_robot_ contains the urdf files, communication message format and kinematics plugins of UR robots.
ur_modern_driver_ is the UR robot ROS driver, which is in charge of publishing robot state,
sending the MoveIt generated joint trajectory message to the robot factory controller.

.. code-block:: bash

  cd <catkin_workspace>/src

  # ROS-Industrial
  git clone -b kinetic-devel https://github.com/ros-industrial/industrial_core.git

  # universal_robot
  git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git

  # ur_modern_driver
  git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git

  cd .. && catkin build

.. _ROS-Industrial: http://wiki.ros.org/Industrial
.. _universal_robot: http://wiki.ros.org/universal_robot
.. _ur_modern_driver: https://github.com/ros-industrial/ur_modern_driver/tree/kinetic-devel

Verify Setup
--------------
You can verify the UR5 setup by running:

.. code-block:: bash

  # terminal 1
  roslaunch ur_description ur5_upload.launch

  # terminal 2
  roslaunch ur_description test.launch

at the left ``Displays`` panel of Rviz, change the ``fixed frame`` to "base_link", and add ``RobotModel`` and ``TF``,
then you can see UR5 brought up in Rviz.

If you would like to see UR5 brought up in Gazebo, run:

.. code-block:: bash

  roslaunch ur_gazebo ur5.launch
