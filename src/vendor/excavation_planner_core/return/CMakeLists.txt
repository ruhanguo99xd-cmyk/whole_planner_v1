cmake_minimum_required(VERSION 3.8)
project(return)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(truck_perceive REQUIRED)
find_package(tra_planning REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/NloptSupport.cmake)

add_executable(
     return 
     src/main.cpp
     src/function_set_dh.cpp
     src/function_set_spline.cpp
     src/function_set_time.cpp
     src/para.cpp
)

# 头文件路径（包含nlopt子目录）
target_include_directories( return  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}        # 基础头文件目录
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/nlopt  # nlopt子目录
    ${EIGEN3_INCLUDE_DIRS}
)

# 使用ment_target_dependencies链接ROS库
ament_target_dependencies(return
    rclcpp
    geometry_msgs
    std_msgs
    truck_perceive
    tra_planning
)

integrated_mission_link_nlopt(return)

# 安装配置
# 1.安装可执行目标
install(TARGETS 
  return
  DESTINATION lib/${PROJECT_NAME})

# 2.安装头文件目录
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})


ament_package()
