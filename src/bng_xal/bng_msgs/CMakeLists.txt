cmake_minimum_required(VERSION 3.5)
project(bng_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BasicStateMsg.msg"
  "msg/GtStateMsg.msg"
  "srv/ExecuteRequest.srv"
  "srv/StartLogger.srv"
  "srv/StopLogger.srv"
  # "msg/VehicleStatus.msg"
  # "msg/ScenarioState.msg"
  # "srv/Spawn.srv"
  # "srv/Control.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)

ament_package()
