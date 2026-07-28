cmake_minimum_required(VERSION 3.0.2)
project(a1_exploration)

find_package(catkin REQUIRED COMPONENTS
  a1_navigation_interfaces
  actionlib
  diagnostic_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    a1_navigation_interfaces
    actionlib
    diagnostic_msgs
    geometry_msgs
    move_base_msgs
    nav_msgs
    rospy
    std_msgs
    tf2_ros
    visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/frontier_explorer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_frontier_core.py)
endif()
