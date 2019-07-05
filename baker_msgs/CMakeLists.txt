cmake_minimum_required(VERSION 2.8.3)
project(baker_msgs)

set(catkin_RUN_PACKAGES
	actionlib
	actionlib_msgs
	sensor_msgs
	std_msgs
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	message_generation
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'action' folder
add_action_files(
	DIRECTORY 
		action
	FILES 
		DirtDetection.action
)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY
		msg
	FILES 
		RotatedRect.msg
		CleanPatternParams.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY
		srv
	FILES 
		GetMap.srv
		CleanPattern.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		actionlib_msgs
		sensor_msgs
		std_msgs
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
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
		message_runtime
)


#catkin_package(
#	CATKIN_DEPENDS
#		message_runtime
#		sensor_msgs
#		std_msgs
#)

#include_directories(
#	${catkin_INCLUDE_DIRS}
#)
