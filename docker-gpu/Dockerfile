FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu20.04


ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai


# 基础工具
RUN apt update && apt install -y \
    software-properties-common \
    curl \
    wget \
    git \
    vim \
    nano \
    unzip \
    ca-certificates \
    gnupg \
    lsb-release \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*


# ROS Noetic软件源
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros-latest.list'


RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | apt-key add -


# ROS Noetic
RUN apt update && apt install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-catkin-tools \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*


# Gazebo11 + ROS控制相关依赖
RUN apt update && apt install -y \
    gazebo11 \
    libgazebo11-dev \
    ros-noetic-gazebo-ros-control \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-xacro \
    liblcm-dev \
    && rm -rf /var/lib/apt/lists/*


# 初始化ROS环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc


# CUDA路径
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}


WORKDIR /workspace
