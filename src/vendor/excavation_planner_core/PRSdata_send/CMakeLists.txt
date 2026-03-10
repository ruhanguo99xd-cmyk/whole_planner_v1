cmake_minimum_required(VERSION 3.8)
project(PRSdata_send)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(point_cloud_processing_pkg REQUIRED)
find_package(truck_perceive REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shovel_interfaces REQUIRED)
find_package(Eigen3 REQUIRED) 

add_executable(prsdata_server  src/data_server.cpp)
ament_target_dependencies(prsdata_server rclcpp point_cloud_processing_pkg geometry_msgs)

add_executable(perceive_truck_server src/perceive_truck_server.cpp)
ament_target_dependencies(perceive_truck_server rclcpp truck_perceive geometry_msgs)

add_executable(prsdata_client src/prsdata_client.cpp)
ament_target_dependencies(prsdata_client rclcpp rclcpp_action shovel_interfaces)

target_include_directories( prsdata_server  PRIVATE
    ${EIGEN3_INCLUDE_DIR} 
)

install(TARGETS
  prsdata_server
  perceive_truck_server
  prsdata_client
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
