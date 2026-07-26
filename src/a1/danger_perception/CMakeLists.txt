cmake_minimum_required(VERSION 3.0.2)
project(a1_danger_perception)

find_package(catkin REQUIRED COMPONENTS
  a1_navigation_interfaces
  cv_bridge
  geometry_msgs
  message_filters
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    a1_navigation_interfaces
    cv_bridge
    geometry_msgs
    message_filters
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/danger_perception_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
