cmake_minimum_required(VERSION 3.16)
project(fs_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Spot.msg"
  "msg/Mission.msg"
  "msg/PlantRecord.msg"
  "msg/HeadStatus.msg"
  "msg/StabilityStatus.msg"
  "msg/TractionStatus.msg"
  "msg/RobotState.msg"
  "msg/SafetyStatus.msg"
  "msg/HwSafety.msg"
  "srv/ReserveBay.srv"
  "action/PlantSpot.action"
  "action/TenderService.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
