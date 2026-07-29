cmake_minimum_required(VERSION 3.0.2)
project(a1_navigation_tests)

find_package(catkin REQUIRED COMPONENTS
  a1_building_behavior
  a1_exploration
  a1_floor_mapping
  a1_localization
  a1_navigation_interfaces
  actionlib
  actionlib_msgs
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  move_base_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    a1_building_behavior
    a1_exploration
    a1_floor_mapping
    a1_localization
    a1_navigation_interfaces
    actionlib
    actionlib_msgs
    diagnostic_msgs
    dynamic_reconfigure
    geometry_msgs
    move_base_msgs
    nav_msgs
    rosgraph
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/fake_obstacle_publisher.py
  scripts/cmd_mux_acceptance.py
  scripts/constant_velocity_stability_diagnostic.py
  scripts/dev_truth_localization_bridge.py
  scripts/indoor_start_frontier_explorer.py
  scripts/indoor_start_runner.py
  scripts/single_floor_exploration_client.py
  scripts/single_floor_gazebo_acceptance.py
  scripts/safe_stop_short_test.py
  scripts/stability_bag_analyzer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/run_indoor_start_once.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_gazebo_joint_limit_config.py)
  catkin_add_nosetests(test/test_indoor_start_contract.py)
  catkin_add_nosetests(test/test_indoor_start_runner.py)
  catkin_add_nosetests(test/test_single_floor_acceptance_helpers.py)
  catkin_install_python(PROGRAMS
    test/exploration_runtime_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/exploration_runtime.test)
endif()
