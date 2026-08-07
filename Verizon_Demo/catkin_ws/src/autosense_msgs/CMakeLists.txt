cmake_minimum_required(VERSION 2.8.3)
project(autosense_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  TrackingObjectArray.msg
  TrackingObjectArray_local_frame.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs
)
