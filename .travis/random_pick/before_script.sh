#!/bin/bash

# Set environment variables
python_version=""
. /opt/intel/computer_vision_sdk/bin/setupvars.sh

# Extend cmake args
CMAKE_ARGS="$CMAKE_ARGS -DUSE_CAFFE=OFF -DUSE_OPENVINO=ON"

# Enable random_pick build
CMAKE_ARGS="$CMAKE_ARGS -DBUILD_RANDOM_PICK=ON"