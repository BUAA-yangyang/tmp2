cmake_minimum_required(VERSION 3.0.2)
project(a1_result_manager)

find_package(catkin REQUIRED COMPONENTS
  a1_navigation_interfaces
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    a1_navigation_interfaces
    rospy
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/result_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
