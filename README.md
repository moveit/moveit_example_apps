# moveit_example_apps

This project demonstrates advanced applications for using MoveIt with full robotic setups. For each listed application, there is a tutorial introducing how to replicate and run the application either in simulation or with real robot. If you are interested in the technique details, you can visit [MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/) for further information.

## Applications

| Application Name | Documentation          | Docker                                     |
| ---------------- | -----------------------| ------------------------------------------ |
| Random Pick      | [random_pick_tutorial] | [moveit/moveit_example_apps:random_pick]() |

[random_pick_tutorial]: https://roboticsyy.github.io/moveit_example_apps/doc/random_pick/random_pick.html

## Continuous Integration

[![Build Status](https://travis-ci.org/ros-planning/moveit_example_apps.svg?branch=master)](https://travis-ci.org/ros-planning/moveit_example_apps)

## Contributing

There are many diverse application examples of what you can use MoveIt for. Your contribution is encouraged, no matter the application is developed for some usages, competitions or research topics, or to demonstrate the newly developed MoveIt feature.

If you are interested in adding a new MoveIt application in this project, please consider following the below instructions.

### Directory Structure

For each example application, it should contain two parts: example codes and tutorials.

* The example codes of each application should live in its own subdirectory in the `./examples/<app_name>` directory.
* Under `./examples/<app_name>` directory, there should be two packages: `<app_name>` and `<app_moveit_config_package>`.
* Refer to the example below for how the structure of `./examples/<app_name>` directory would look like:

```
moveit_example_apps/examples/
└── <app_name>/
    ├── <app_name>/
    |   ├── control/                             # Launch and config files of controllers
    |   ├── gazebo/                              # Gazebo launch files
    |   ├── launch/                              # Demo launch files
    |   ├── rviz/                                # Rviz config files
    |   ├── scripts/                             # Python files
    |   ├── data/                                # Custom simulation data
    |   |   └── <app_name>.pcd
    |   ├── include/                             # Header files
    |   |   └── <app_name>.h
    |   ├── src/                                 # Source files
    |   |   └── <app_name>.cpp
    |   ├── urdf/                                # URDF and mesh files
    |   ├── CMakeLists.txt
    |   ├── package.xml
    |   └── README.md
    └── <app_moveit_config_package>/             # Custom moveit_config package
```

* The tutorials build with [rosdoc_lite](http://wiki.ros.org/rosdoc_lite) and [Sphinx](http://wiki.ros.org/Sphinx) builder. Each tutorial should live in its own subdirectory under the `./tutorials/doc/<app_name>` directory.
* Each tutorial should use the [reStructuredText](http://www.sphinx-doc.org/en/stable/rest.html) format and follow the structure of `./tutorials/doc/template.rst`.
* Add your tutorial file name to `./tutorials/index.rst`.
* Refer to the example below for how the structure of `./tutorials` directory would look like:

```
moveit_example_apps/tutorials/
├── _static/
├── doc/
|   ├── <app_name>/
|   |   ├── images/                              # Screen shots of app outputs
|   |   └── <app_name>.rst                       # Custom app tutorial
|   └── template.rst                             # App tutorial template
├── index.rst                                    # Entry file of moveit_example_apps' tutorials
├── conf.py
├── build_locally.sh
├── rosdoc.yaml
├── README.md
├── CMakeLists.txt
└── package.xml
```

### CI Test

In order to guarantee a successful compilation and running at others' side, it is necessary to make the application continuously tested. The application contributor can consider providing the following components as what `random_pick` did to enable moveit_ci to the application:

- `.docker/random_pick/Dockerfile`: for setting up the dependencies of external libraries, which overlay on moveit/moveit:master-source.
- `.rosinstall/random_pick/random_pick.rosinstall`: for the source dependencies required to be built in the catkin workspace.
- `.travis/random_pick/before_script.sh`: for setting up the environment variables and directory path of dependencies in the docker container.
- A custom docker image, built from the docker file. Perhaps this image can be tagged as `moveit/moveit_example_apps:random_pick`.

