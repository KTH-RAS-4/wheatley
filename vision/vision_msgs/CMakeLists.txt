cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs message_generation
)

add_message_files(
  FILES
  Object.msg
  Objects.msg
  PreprocessedClouds.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS message_runtime
  DEPENDS
)
