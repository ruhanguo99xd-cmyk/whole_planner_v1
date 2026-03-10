cmake_minimum_required(VERSION 3.8)
project(planner_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

###############
# srv 生成
###############
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGoal.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

###############
# Node
###############
add_executable(planner_client_node src/planner_client.cpp)

ament_target_dependencies(planner_client_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  rosidl_default_runtime
)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(planner_client_node
  ${typesupport_target})

  ###############
install(TARGETS planner_client_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

###############
ament_package()
###############
