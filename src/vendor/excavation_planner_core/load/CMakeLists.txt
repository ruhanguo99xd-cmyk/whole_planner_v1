cmake_minimum_required(VERSION 3.8)
project(load)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(truck_perceive REQUIRED)
find_package(tra_planning REQUIRED)

add_executable(
     load 
     src/main.cpp
     src/function_set_dh.cpp
     src/function_set_spline.cpp
     src/function_set_other.cpp
     src/para.cpp
)

# 头文件路径（包含nlopt子目录）
target_include_directories( load  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}        # 基础头文件目录
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/nlopt  # nlopt子目录
    ${EIGEN3_INCLUDE_DIRS}
)

# 使用ment_target_dependencies链接ROS库
ament_target_dependencies(load
    rclcpp
    geometry_msgs
    std_msgs
    truck_perceive
    tra_planning
)

# 使用target_link_libraries链接非ROS库
target_link_libraries(load
    /usr/lib/x86_64-linux-gnu/libnlopt.so  
)

# 安装配置
# 1.安装可执行目标
install(TARGETS 
  load
  DESTINATION lib/${PROJECT_NAME})

# 2.安装头文件目录
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_package()
