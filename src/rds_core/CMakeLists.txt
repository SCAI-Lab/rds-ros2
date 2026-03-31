cmake_minimum_required(VERSION 3.8)
project(rds_core)

# Use C++17 (or C++14 if needed)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rvo2_lib REQUIRED)
find_package(SDL2 REQUIRED)


add_library(${PROJECT_NAME}
  src/capsule.cpp
  src/distance_minimizer.cpp
  src/rds_5.cpp
  src/rds_5_agent.cpp
  src/my_rvo.cpp
  src/gui.cpp
  src/window.cpp
)

target_include_directories(rds_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(${PROJECT_NAME}
  PUBLIC
    rvo2_lib::rvo2_lib
    SDL2::SDL2
)


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rvo2_lib SDL2)

ament_package()
