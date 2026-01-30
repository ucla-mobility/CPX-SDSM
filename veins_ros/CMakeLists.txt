cmake_minimum_required(VERSION 3.8)
project(sdsm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/DDateTime.msg"
  "msg/Position3D.msg"
  "msg/PositionOffsetXYZ.msg"
  "msg/PositionConfidenceSet.msg"
  "msg/VehicleSize.msg"
  "msg/ObstacleSize.msg"
  "msg/DetectedObjectCommonData.msg"
  "msg/DetectedVehicleData.msg"
  "msg/DetectedVRUData.msg"
  "msg/DetectedObstacleData.msg"
  "msg/DetectedObjectData.msg"
  "msg/SensorDataSharingMessage.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
