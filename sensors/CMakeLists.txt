cmake_minimum_required(VERSION 2.8.3)
project(sensors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  ras_arduino_msgs
  visualization_msgs
  std_msgs
  nav_msgs
  tf
  pcl_ros
  sensor_msgs
  occupancy_grid_utils
  wheatley_common
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Distance.msg
  SensorClouds.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ras_arduino_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime ras_arduino_msgs roscpp visualization_msgs std_msgs nav_msgs tf occupancy_grid_utils wheatley_common
#  INCLUDE_DIRS include
#  LIBRARIES sensors
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(sensors
#   src/${PROJECT_NAME}/sensors.cpp
# )

## Declare a cpp executable
add_executable(pose src/pose_node.cpp)
add_executable(ekf src/ekf_node.cpp)
add_executable(ir_distances src/ir_distances_node.cpp)
add_executable(ir_points src/ir_points_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pose pose_generate_messages_cpp)
add_dependencies(ekf pose_generate_messages_cpp)
add_dependencies(ir_distances pose_generate_messages_cpp)
add_dependencies(ir_points pose_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pose
  ${catkin_LIBRARIES}
)

target_link_libraries(ekf
  ${catkin_LIBRARIES}
)

target_link_libraries(ir_distances
  ${catkin_LIBRARIES}
)

target_link_libraries(ir_points
  ${catkin_LIBRARIES}
)
