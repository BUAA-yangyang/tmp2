cmake_minimum_required(VERSION 3.0.2)
project(a1_mission_manager)
find_package(catkin REQUIRED COMPONENTS
  actionlib a1_navigation_interfaces building_generator_interfaces
  diagnostic_msgs geometry_msgs move_base_msgs nav_msgs rospy std_msgs
  std_srvs visualization_msgs)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/align_to_opening.py
  scripts/corridor_axis.py
  scripts/elevator_exit.py
  scripts/multifloor_mission_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_corridor_axis.py)
  catkin_add_nosetests(test/test_elevator_exit.py)
  catkin_add_nosetests(test/test_option_a_contract.py)
endif()
