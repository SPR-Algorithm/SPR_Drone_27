cmake_minimum_required(VERSION 3.16)
project(drone_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EstimatorStatus.msg" "msg/FlightStatus.msg"
  "msg/TrajectorySetpoint.msg" "msg/MissionState.msg"
  DEPENDENCIES std_msgs geometry_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # The ROS generator replaces this target's directory-level warning options.
  target_compile_options(${PROJECT_NAME}__rosidl_typesupport_c
    PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
