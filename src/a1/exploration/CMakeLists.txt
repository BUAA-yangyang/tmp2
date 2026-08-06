cmake_minimum_required(VERSION 3.0.2)
project(a1_exploration)

find_package(catkin REQUIRED COMPONENTS
  a1_navigation_interfaces
  actionlib
  diagnostic_msgs
  dynamic_reconfigure
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
    dynamic_reconfigure
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
  # Every test file in test/ is registered here on purpose: they were
  # only ever run by hand before, so nothing in the build would have
  # caught a regression in them.
  catkin_add_nosetests(test/test_frontier_core.py)
  catkin_add_nosetests(test/test_final_zero.py)
  catkin_add_nosetests(test/test_entry_speed_limit.py)
  catkin_add_nosetests(test/test_permanent_unreachable.py)
  catkin_add_nosetests(test/test_progress_watchdog.py)
  catkin_add_nosetests(test/test_corridor_gate.py)
  catkin_add_nosetests(test/test_wall_backstop.py)
  catkin_add_nosetests(test/test_interstitial_zero.py)
  catkin_add_nosetests(test/test_return_anchor.py)
  catkin_add_nosetests(test/test_near_field_gate.py)
  catkin_add_nosetests(test/test_room_axis_bounds.py)
  catkin_add_nosetests(test/test_room_queue.py)
  catkin_add_nosetests(test/test_room_completion.py)
  catkin_add_nosetests(test/test_room_source_keepout_contract.py)
  catkin_add_nosetests(test/test_room_frontier_diagnostics.py)
  catkin_add_nosetests(test/test_floor_state_contract.py)
  catkin_add_nosetests(test/test_no_progress_yaw_gate.py)
endif()
