GPD Setup
=========

This section provides some special notices on the installation of the `Grasp Pose Detection <https://github.com/atenpas/gpd>`_
and its dependencies.

System Requirement
-------------------
* Ubuntu Linux 18.04 on 64-bit
* `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_.

`Grasp Pose Detection` can be build either on Caffe or OpenVINO. Please follow below instructions to install one of them.

Install Caffe from source
-------------------------
Before installing Caffe, make sure the dependencies are properly installed. It is recommended to run the following command:

.. code-block:: bash

   sudo apt build-dep caffe-cpu        # dependencies for CPU-only version
   sudo apt build-dep caffe-cuda       # dependencies for CUDA version

It requires a deb-src line in your sources.list. You can enable this by selecting `Source code` on the first tab
of Ubuntu `Software & Updates` **OR** running the commands:

.. code-block:: bash

   sudo sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list
   sudo apt-get update

.. attention:: caffe builds on `gflags` and `glog`. If you follow the above method to install these two dependencies,
               the header files and libraries are distributed under `/usr/inlcude` and `usr/lib/x86_64-linux-gnu`.
               If you installed these two libraries from source, the headers and libraries are distributed under
               `/usr/local/include` and `/usr/local/lib`. Make sure you have installed these two libraries in one way.
               Otherwise, there could be runtime error for caffe.

Clone the Caffe repository into some folder:

.. code-block:: bash

   mkdir ~/git && cd ~/git
   git clone https://github.com/BVLC/caffe.git && cd caffe

Follow `Cmake Build instructions <http://caffe.berkeleyvision.org/installation.html#compilation>`_ to build caffe.

.. note:: caffe builds on OpenCV 3.x. If there are both OpenCV 4.x and 3.x on your system, make sure you have added
          version info like `find_package(OpenCV 3.x.x)` in the `caffe/cmake/Dependencies.cmake`.

Install OpenVINO
----------------
Follow this tutorial `Install OpenVINO toolkit <https://github.com/atenpas/gpd/blob/master/tutorials/tutorial_openvino.md#1-install-openvino-toolkit>`_.

Install Grasp Pose Generator (GPG)
----------------------------------
Clone the `GPG <https://github.com/atenpas/gpg>`_ repository into some folder:

.. code-block:: bash

   cd <location_of_your_workspace>
   git clone https://github.com/atenpas/gpg.git

Build and install the GPG:

.. code-block:: bash

   cd gpg
   mkdir build && cd build
   cmake ..
   make
   sudo make install

Install Grasp Pose Detection (GPD)
----------------------------------

Clone the `GPD <https://github.com/atenpas/gpd>`_ repository into some folder:

.. code-block:: bash

   cd <location_of_your_workspace/src>
   git clone -b libgpd https://github.com/sharronliu/gpd.git

**To build GPD with Caffe**, add below lines in front of `find_package(Caffe)` in the `CMakeLists.txt` of gpd:

.. code-block:: cmake

    set(Caffe_DIR ~/git/caffe/build)

    find_path(Caffe_INCLUDE_DIRS NAMES caffe/caffe.hpp caffe/common.hpp caffe/net.hpp caffe/proto/caffe.pb.h caffe/util/io.hpp caffe/vision_layers.hpp
    HINTS
    ${Caffe_DIR}/include)

    find_library(Caffe_LIBRARIES NAMES caffe
    HINTS
    ${Caffe_DIR}/lib)

**To build GPD with OpenVINO**, follow `this <https://github.com/atenpas/gpd/blob/master/tutorials/tutorial_openvino.md#3-build-gpd-with-openvino>`_ tutorial.

Build GPD:

.. code-block:: bash

   cd <location_of_your_workspace>
   catkin build

Verify GPD installation by running the `GPD Tutorials <https://github.com/atenpas/gpd#6-tutorials>`_
