# moveit_app_tutorials

This package contains the tutorial of each application in `moveit_example_apps/examples/`.

The source files of the tutorial, which are `.rst` files, locate in the `./doc` folder. These tutorials are created with [rosdoc_lite](http://wiki.ros.org/rosdoc_lite) tool using [sphinx](http://www.sphinx-doc.org/en/master/index.html) engine.

In order to build and read the tutorials, the user needs to install `rosdoc_lite` and `sphinx` at first.

Install `rosdoc_lite`:

```sh
sudo apt-get install ros-<rosversion>-rosdoc-lite
```

Install `Sphinx`:

```sh
sudo apt-get install python-sphinx
```

Build the document:

```sh
source path_to_your_ros_workspace/devel/setup.bash

roscd moveit_app_tutorials

rosdoc_lite -o build .
```

If the document files are built successfully, please open the file `./build/html/index.html` with a regular web browser to see the
tutorials.
