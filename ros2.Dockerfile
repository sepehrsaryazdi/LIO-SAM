FROM osrf/ros:humble-desktop-full-jammy

RUN apt-get update \
    && apt-get install -y ros-humble-navigation2 \
    && apt-get install -y ros-humble-robot-localization \
    && apt-get install -y ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt install -y software-properties-common \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.2 \
    && apt-get update \
    && apt install -y libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN mkdir -p ~/ros2_ws/src \ 
    && cd ~/ros2_ws/src \
    && git clone https://github.com/sepehrsaryazdi/LIO-SAM.git \
    && source /opt/ros/humble/setup.bash

# RUN mkdir -p ~/catkin_ws/src \
#     && cd ~/catkin_ws/src \
#     && git clone https://github.com/sepehrsaryazdi/LIO-SAM.git \
#     && cd .. \
#     && source /opt/ros/kinetic/setup.bash \
#     && catkin_make

# RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc \
#     && echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# WORKDIR /root/catkin_ws
