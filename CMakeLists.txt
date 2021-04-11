cmake_minimum_required(VERSION 3.0.2)
project(kamayuc_localization)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2_ros
  robot_localization  
  ar_track_alvar
  ar_track_alvar_msgs
  nav_msgs
  geometry_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kamayuc_localization
#  CATKIN_DEPENDS robot_localization
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable

add_executable(map_artag_ekf src/map_artag_ekf.cpp)
add_executable(setup_map_odom src/setup_map_odom.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(map_artag_ekf
  ${catkin_LIBRARIES}
)
target_link_libraries(setup_map_odom
  ${catkin_LIBRARIES}
)