cmake_minimum_required(VERSION 3.5)
project(description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 找到 package 的共享目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
