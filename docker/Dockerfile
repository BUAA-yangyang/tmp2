FROM osrf/ros:noetic-desktop-full-focal


ENV DEBIAN_FRONTEND=noninteractive


RUN apt update && apt install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    wget \
    curl \
    vim \
    liblcm-dev


RUN apt update && apt install -y \
    gazebo11 \
    libgazebo11-dev \
    ros-noetic-gazebo-ros-control \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-xacro


RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
