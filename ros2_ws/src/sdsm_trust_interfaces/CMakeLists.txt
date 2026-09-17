cmake_minimum_required(VERSION 3.8)
project(sdsm_trust_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sdsm_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReceivedSdsm.msg"
  "msg/TrustVerdict.msg"
  DEPENDENCIES sdsm_msgs
)

ament_export_dependencies(rosidl_default_runtime sdsm_msgs)

ament_package()
