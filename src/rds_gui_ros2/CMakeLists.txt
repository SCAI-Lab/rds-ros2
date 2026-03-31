cmake_minimum_required(VERSION 3.8)
project(rds_gui_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rvo2_lib REQUIRED)
find_package(rds_core REQUIRED)
find_package(rds_msgs REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
)

add_executable(rds_gui_node
  src/rds_gui_ros_node.cpp
)

target_include_directories(rds_gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rds_gui_node
  rclcpp
  rds_core
  rds_msgs
)

target_link_libraries(rds_gui_node
  ${SDL2_LIBRARIES}
)

install(TARGETS rds_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
