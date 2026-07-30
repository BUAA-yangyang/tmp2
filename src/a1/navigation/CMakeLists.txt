cmake_minimum_required(VERSION 3.0.2)
project(a1_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/scan_to_obstacle_cloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 本包目前是纯配置/启动包（move_base 是现成节点，无需自己写 C++）。
# 安装 launch/config/rviz，便于 $(find a1_navigation) 在任何工作空间下解析。
install(DIRECTORY launch config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
