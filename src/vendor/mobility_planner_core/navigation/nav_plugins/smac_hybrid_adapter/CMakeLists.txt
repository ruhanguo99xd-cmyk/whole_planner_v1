cmake_minimum_required(VERSION 3.8)
project(smac_hybrid_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/smac_hybrid_adapter.cpp
)

ament_target_dependencies(${library_name}
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  nav2_smac_planner
)

pluginlib_export_plugin_description_file(nav2_core plugin_description.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES params.yaml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  nav2_smac_planner
)

ament_package()
