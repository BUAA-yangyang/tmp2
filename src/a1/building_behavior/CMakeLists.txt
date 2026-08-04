cmake_minimum_required(VERSION 3.0.2)
project(a1_building_behavior)

find_package(catkin REQUIRED COMPONENTS
  a1_navigation_interfaces
  actionlib
  building_generator_interfaces
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    a1_navigation_interfaces
    actionlib
    building_generator_interfaces
    rospy
)

catkin_install_python(PROGRAMS
  scripts/building_behavior_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_public_scene.py)
endif()
