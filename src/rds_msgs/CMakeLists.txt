cmake_minimum_required(VERSION 3.8)
project(rds_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Message files
set(msg_files
  "msg/VelocityCommand.msg"
  "msg/VelocityCommandHexagonLimits.msg"
  "msg/Point2D.msg"
  "msg/HalfPlane2D.msg"
  "msg/Circle.msg"
  "msg/Capsule.msg"
  "msg/ToGui.msg"
  "msg/PedestrianTrack.msg"
  "msg/PedestrianTracks.msg"
)

# Service files
set(srv_files
  "srv/VelocityCommandCorrectionRDS.srv"
)

# Register interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

# Install interfaces to make headers available
install(DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_export_include_directories("include")
#ament_export_libraries(${PROJECT_NAME})
ament_package()