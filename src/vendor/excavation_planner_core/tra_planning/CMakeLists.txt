cmake_minimum_required(VERSION 3.8)
project(tra_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)    # 用了std_msgs/Bool
find_package(point_cloud_processing_pkg REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)


# 添加可执行文件
add_executable(
    trajectory_planner 
    src/main.cpp
    src/optimization.cpp
    src/parameters.cpp
    src/preplan.cpp
    src/trajectory_to_VandF.cpp
)

# 头文件路径（包含nlopt子目录）
target_include_directories( trajectory_planner  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}        # 基础头文件目录
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/nlopt  # nlopt子目录
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# 添加PCL定义
add_definitions(${PCL_DEFINITIONS})

# 使用ment_target_dependencies链接ROS库
ament_target_dependencies(trajectory_planner 
    rclcpp
    std_msgs
    point_cloud_processing_pkg
    rosidl_default_runtime
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExcavationInfo.msg"
)

rosidl_target_interfaces(trajectory_planner
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# 使用target_link_libraries链接非ROS库
target_link_libraries(trajectory_planner
    ${PCL_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libnlopt.so  
)

# 安装配置
# 1.安装可执行目标
install(TARGETS 
  trajectory_planner 
  DESTINATION lib/${PROJECT_NAME}
  )

# 2.安装头文件目录
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# 安装data目录下的所有文件，数据文件只能通过编译后的share文件共享
install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()