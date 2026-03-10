cmake_minimum_required(VERSION 3.8)
project(integrated_mission_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WalkMission.action"
  "action/DigMission.action"
  "msg/PlannerMode.msg"
  "msg/PlcSnapshot.msg"
  "msg/SubsystemStatus.msg"
  "srv/SubmitMission.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
