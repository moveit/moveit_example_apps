# moveit/moveit_example_apps:random_pick
From moveit/moveit:master-source

MAINTAINER Yu Yan yu.yan@intel.com

ARG http_proxy
ENV DEPEND /root/depends
RUN mkdir -p $DEPEND
RUN apt-get update && apt-get install -y && \
      rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \ 
      vim \
      wget \
      build-essential \
      git \
      cmake \
      python3-pip \
      python-pip \
      curl \
      gnupg2 \
      lsb-release \
      sudo && \
      rm -rf /var/lib/apt/lists/*

# install openvino 
WORKDIR $DEPEND
RUN mkdir -p openvino_binart && cd openvino_binart && \
    apt-get update && apt-get install -y cpio && \
      rm -rf /var/lib/apt/lists/*
# wget openvino_R5
RUN wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/15078/l_openvino_toolkit_p_2018.5.455.tgz && \
    tar -xvf l_openvino_toolkit_p_2018.5.455.tgz &&rm l_openvino_toolkit_p_2018.5.455.tgz && \
    cd l_openvino_toolkit_p_2018.5.455 && \
    ./install_cv_sdk_dependencies.sh && \
    sed -i 's/ACCEPT_EULA=decline/ACCEPT_EULA=accept/g' silent.cfg && \
    ./install.sh --silent silent.cfg
# install dependencies
RUN cd /opt/intel/computer_vision_sdk/install_dependencies && \
    ./install_NEO_OCL_driver.sh

# install gpg
WORKDIR $DEPEND
RUN git clone --depth=1 https://github.com/atenpas/gpg.git
WORKDIR $DEPEND/gpg
RUN mkdir build
WORKDIR $DEPEND/gpg/build
RUN cmake .. && make && make install
RUN ldconfig

# install librealsense
#RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE &&\
RUN if [ "$http_proxy" != "" ]; \
    then \
      apt-key adv --keyserver keys.gnupg.net \
      --keyserver-options http-proxy=$http_proxy \
      --recv-key C8B3A55A6F3EFCDE ;\
    else \
      apt-key adv --keyserver keys.gnupg.net \
      --recv-key C8B3A55A6F3EFCDE ;\
    fi
RUN apt-get update && apt-get install -y python3-software-properties software-properties-common && \
       rm -rf /var/lib/apt/lists/*
RUN add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u && \
    apt-get update && apt-get install -y librealsense2-dkms \
      librealsense2-utils \
      librealsense2-dev \
      librealsense2-dbg && \
       rm -rf /var/lib/apt/lists/*

# fix moveit code
WORKDIR /root/ws_moveit
RUN number=`sed -n '/if (callIK(ik_query, adapted_ik_validity_callback, ik_timeout_, state, project && a == 0))/=' ./src/moveit/moveit_core/constraint_samplers/src/default_constraint_samplers.cpp` && \
    echo $number && \
    sed -i "${number}a\    if (callIK(ik_query, adapted_ik_validity_callback, ik_timeout_, state, project || a == 0))" ./src/moveit/moveit_core/constraint_samplers/src/default_constraint_samplers.cpp && \
    sed -i "${number}d" ./src/moveit/moveit_core/constraint_samplers/src/default_constraint_samplers.cpp
RUN catkin build