cmake_minimum_required(VERSION 2.8.3)
project(baker_detections_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# set catkin run package
SET(CATKIN_RUN_PACKAGES
  actionlib
  geometry_msgs
  cob_object_detection_msgs
  roslib
  rospy
  std_msgs
  tf
  visualization_msgs
)

# set cakin build package
SET(CATKIN_BUILD_PACKAGES
  ${CATKIN_RUN_PACKAGES}
  cmake_modules
  )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)

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
  LIBRARIES fake_trash_dirt_detection
  CATKIN_DEPENDS ${CATKIN_RUN_PACKAGES}
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
)

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
