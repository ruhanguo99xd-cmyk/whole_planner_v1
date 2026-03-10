cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
