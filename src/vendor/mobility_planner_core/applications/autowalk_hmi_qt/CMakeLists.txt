cmake_minimum_required(VERSION 3.8)
project(autowalk_hmi_qt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(autowalk_hmi_qt
  include/autowalk_hmi_qt/map_canvas_widget.hpp
  include/autowalk_hmi_qt/main_window.hpp
  include/autowalk_hmi_qt/track_speed_plot_widget.hpp
  src/map_canvas_widget.cpp
  src/main.cpp
  src/main_window.cpp
  src/track_speed_plot_widget.cpp
)

target_include_directories(autowalk_hmi_qt PRIVATE include)

target_link_libraries(autowalk_hmi_qt
  Qt5::Widgets
)

ament_target_dependencies(autowalk_hmi_qt
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS autowalk_hmi_qt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
