cmake_minimum_required(VERSION 3.0.2)
project(a1_navigation_interfaces)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  CmdMuxStatus.msg
  DangerDetection.msg
  DangerDetectionArray.msg
  DangerSource.msg
  DangerSourceArray.msg
  ExplorationStatus.msg
  FloorStatus.msg
  MissionStatus.msg
  SearchRegion.msg
  SearchRegionArray.msg
  WallSegment.msg
  WallSegmentArray.msg
  Doorway.msg
  DoorwayArray.msg
)

add_service_files(
  FILES
  SetMissionMode.srv
)

add_action_files(
  FILES
  ExploreFloor.action
  SpecialBehavior.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    std_msgs
)
