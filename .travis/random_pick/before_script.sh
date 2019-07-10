#!/bin/bash
export DEPEND=/root/depends
mkdir -p $DEPEND

# Install OpenVINO
cd $DEPEND
mkdir -p openvino_binart && cd openvino_binart
apt-get update && apt-get install -y cpio
## wget openvino_R5
# wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/15078/l_openvino_toolkit_p_2018.5.455.tgz
# tar -xvf l_openvino_toolkit_p_2018.5.455.tgz && rm l_openvino_toolkit_p_2018.5.455.tgz
# cd l_openvino_toolkit_p_2018.5.455
## Install git large file storage
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
apt-get install git-lfs
git lfs install
## Download OpenVINO
git lfs clone https://github.com/RoboticsYY/openvino_install_resource.git && cd openvino_install_resource
tar -xvf l_openvino_toolkit_p_2018.5.455.tgz && rm l_openvino_toolkit_p_2018.5.455.tgz
cd l_openvino_toolkit_p_2018.5.455
## Install OpenVINO
./install_cv_sdk_dependencies.sh
sed -i 's/ACCEPT_EULA=decline/ACCEPT_EULA=accept/g' silent.cfg
./install.sh --silent silent.cfg
## Install dependencies
cd /opt/intel/computer_vision_sdk/install_dependencies
./install_NEO_OCL_driver.sh
# Set environment variables
python_version=""
. /opt/intel/computer_vision_sdk/bin/setupvars.sh

# Install gpg
cd $DEPEND
git clone --depth=1 https://github.com/atenpas/gpg.git
cd gpg && mkdir build && cd build
cmake .. && make && make install

# Extend cmake args
CMAKE_ARGS="$CMAKE_ARGS -DUSE_CAFFE=OFF -DUSE_OPENVINO=ON"

# Enable random_pick build
CMAKE_ARGS="$CMAKE_ARGS -DBUILD_RANDOM_PICK=ON"