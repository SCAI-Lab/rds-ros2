cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)

project(rds_ros2)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_msgs REQUIRED)
find_package(rds_core REQUIRED)
find_package(rvo2_lib REQUIRED) 
find_package(rds_msgs REQUIRED) 

# Debug messages
message(STATUS "rds_msgs INCLUDE_DIRS: ${rds_msgs_INCLUDE_DIRS}")
message(STATUS "rds_msgs LIBRARIES: ${rds_msgs_LIBRARIES}")

# ========================================
# LIBRARIES
# ========================================

# Base wrapper library
add_library(wrapper_base
  src/wrapper_base.cpp
)

target_include_directories(wrapper_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wrapper_base
  rclcpp
  geometry_msgs
  rds_msgs
)

# ========================================
# EXECUTABLES
# ========================================

# Main RDS node
add_executable(rds_node src/rds_node.cpp)

target_include_directories(rds_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rds_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  nav_msgs
  rds_core
  rds_msgs
  rvo2_lib  
)

target_link_libraries(rds_node
  wrapper_base
)

# Nominal command node
add_executable(rds_nominal_command_node
  src/nominal_command_node.cpp
)

ament_target_dependencies(rds_nominal_command_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

target_link_libraries(rds_nominal_command_node
  wrapper_base
)

# VoxelGrid Filter Node
add_executable(voxel_filter_node
  src/voxel_filter_node.cpp
)

target_include_directories(voxel_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(voxel_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(voxel_filter_node
  ${PCL_LIBRARIES}
)

# Simple obstacle avoidance node
add_executable(simple_obstacle_avoidance 
  src/simple_obstacle_avoidance.cpp
)

ament_target_dependencies(simple_obstacle_avoidance 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  PCL 
  pcl_conversions
)

target_link_libraries(simple_obstacle_avoidance
  ${PCL_LIBRARIES}
)

# ========================================
# INSTALLATIONS
# ========================================

# Install executables
install(TARGETS
  rds_node
  rds_nominal_command_node
  voxel_filter_node
  simple_obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts with permissions
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install specific Python scripts as programs
install(PROGRAMS
  scripts/fake_laser_publisher.py
  scripts/pipeline_monitor.py
  scripts/scan_to_pointcloud.py
  scripts/pointcloud_concatenate.py
  scripts/improved_fake_laser_publisher.py
  scripts/velocity_visualizer.py
  scripts/test_velocity_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

# Finalize package
ament_package()