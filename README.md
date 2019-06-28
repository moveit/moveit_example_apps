# moveit_example_apps

This project demonstrates advanced applications for using MoveIt with full robotic setups. For each listed application, there is a tutorial introducing how to replicate and run the application either in simulation or with real robot. If you are interested in the technique details, you can visit [MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/) for further information.

## Applications

| Application Name | Documentation          | Docker |
| ---------------- | -----------------------| ------ |
| Random Pick      | [random_pick_tutorial] | WIP    |

[random_pick_tutorial]: https://roboticsyy.github.io/moveit_example_apps/doc/random_pick/random_pick.html

## Continuous Integration

Work in progress.

## Contributing

There are many diverse application examples of what you can use MoveIt for. Your contribution is encouraged, no matter the application is developed for some usages, competitions or research topics, or to demonstrate the newly developed MoveIt feature.

If you are interested in adding a new MoveIt application in this project, please consider following the below instructions.

### Directory Structure

For each example application, it should contain two parts: example codes and tutorials.

* The example codes of each application should live in its own subdirectory in the `./examples/<app_name>` direcotry.
* Under `./examples/<app_name>` direcotry, there should be two packages: `<app_name>` and `<app_moveit_config_package>`.
* Refer to the example below for how the structure of `./examples/<app_name>` directory would look like:

```
moveit_example_apps/examples/
└── <app_name>/
    ├── <app_name>/
    |   ├── control/
    |   |   ├── arm/
    |   |   |   └── ur5_bringup.launch
    |   |   ├── gripper/
    |   |   |   └── hitbot_bringup.launch
    |   |   └── controllers.yaml
    |   ├── gazebo/
    |   |   └── gazebo_bringup.launch
    |   ├── launch/
    |   |   ├── demo/
    |   |   |   ├── camera.launch
    |   |   |   ├── app_fake.launch
    |   |   |   ├── app.launch
    |   |   |   └── robots.launch
    |   |   └── visualize/
    |   |       ├── visualize_arm.launch
    |   |       ├── visualize_gripper.launch
    |   |       └── visualize_customizable_objects.launch
    |   ├── rviz/
    |   |   ├── arm_view.rviz
    |   |   ├── gripper_view.rviz
    |   |   ├── objects_view.rviz
    |   |   └── demo.rviz
    |   ├── scripts/
    |   |   └── gripper_server.py
    |   ├── data/
    |   |   └── <app_name>.pcd
    |   ├── src/
    |   |   └── <app_name>.cpp
    |   ├── urdf/
    |   |   ├── arm/
    |   |   ├── gripper/
    |   |   ├── customizable_objects/  
    |   |   └── meshes/
    |   ├── CMakeLists.txt
    |   ├── package.xml
    |   └── README.md
    └── <app_moveit_config_package>/
```

* The tutorials build with [rosdoc_lite](http://wiki.ros.org/rosdoc_lite) and [Sphinx](http://wiki.ros.org/Sphinx) builder. Each tutorial should live in its own subdirectory under the `./tutorials/doc/<app_name>` direcotry.
* Each tutorial should use the [reStructuredText](http://www.sphinx-doc.org/en/stable/rest.html) format and follow the structure of `./tutorials/doc/template.rst`.
* Add your tutorial file name to `./tutorials/index.rst`.
* Refer to the example below for how the structure of `./tutorials` directory would look like:

```
moveit_example_apps/tutorials/
├── _static/
├── doc/
|   ├── <app_name>/
|   |   ├── images/
|   |   └── <app_name>.rst
|   └── template.rst
├── index.rst
├── conf.py
├── build_locally.sh
├── rosdoc.yaml
├── README.md
├── CMakeLists.txt
└── package.xml
```
