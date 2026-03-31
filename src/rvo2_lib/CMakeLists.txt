cmake_minimum_required(VERSION 3.8)
project(rvo2_lib)

# Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament_cmake for the ROS2 build system
find_package(ament_cmake REQUIRED)

add_compile_options(-Wall -Wextra)

# Build RVO2 as a library
add_library(${PROJECT_NAME} SHARED
  src/Agent.cpp
  src/KdTree.cpp
  src/Obstacle.cpp
  src/RVOSimulator.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install the library and headers
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME} 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Export package info for other packages to find this one
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()