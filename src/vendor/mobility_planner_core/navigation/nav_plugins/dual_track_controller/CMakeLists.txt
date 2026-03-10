cmake_minimum_required(VERSION 3.8)
project(dual_track_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeTrackVel.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(dual_track_controller_node src/dual_track_controller.cpp)
ament_target_dependencies(dual_track_controller_node rclcpp nav_msgs std_msgs geometry_msgs)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(dual_track_controller_node
  ${typesupport_target})

install(TARGETS dual_track_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY srv DESTINATION share/${PROJECT_NAME}/srv)

ament_package()
