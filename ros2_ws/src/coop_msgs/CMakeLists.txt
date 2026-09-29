cmake_minimum_required(VERSION 3.16)
project(coop_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Detection.msg
  msg/DetectionArray.msg
  msg/Track.msg
  msg/TrackArray.msg
  msg/VisibilityTruth.msg
  msg/OverlayTruth.msg
  msg/AgentStatus.msg
  msg/DeviceStatus.msg
  msg/AgentGoal.msg
  msg/GimbalCommand.msg
  msg/GimbalState.msg
  DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
