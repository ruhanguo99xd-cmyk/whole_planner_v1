cmake_minimum_required(VERSION 3.8)
project(app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(
  app
  src/main.cpp
  src/mainwindow.cpp
  include/app/mainwindow.h
  ui/mainwindow.ui
)

set_property(TARGET app PROPERTY AUTOUIC_SEARCH_PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
)
target_include_directories( app  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}        # 基础头文件目录
)

target_link_libraries(app PRIVATE Qt5::Widgets)

install(TARGETS app
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
