cmake_minimum_required(VERSION 3.0.2)
project(a1_cmd_mux)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL)
