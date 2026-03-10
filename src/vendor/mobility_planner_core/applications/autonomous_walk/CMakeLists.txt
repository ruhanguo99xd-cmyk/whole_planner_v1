cmake_minimum_required(VERSION 3.8)
project(autonomous_walk)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. 查找依赖 (必须最先执行) ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(planner_client REQUIRED)

# --- 2. 添加可执行文件 (必须在配置该 target 的属性之前) ---
add_executable(autonomous_walk_node src/autonomous_walk_node.cpp)

# --- 3. 配置 Target 的包含目录 ---
# 这里解决了之前的报错：先有了 autonomous_walk_node，才能指定它的 include
target_include_directories(autonomous_walk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- 4. 绑定依赖库 ---
ament_target_dependencies(autonomous_walk_node
  rclcpp
  rclcpp_action
  nav2_msgs
  nav_msgs
  geometry_msgs
  action_msgs
  std_msgs
  planner_client
)

# --- 5. 安装指令 ---
# 安装二进制文件
install(TARGETS autonomous_walk_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件夹（确保文件夹存在）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 文件夹（确保文件夹存在）
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# --- 6. 测试与打包 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
