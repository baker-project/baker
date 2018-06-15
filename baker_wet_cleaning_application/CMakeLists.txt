cmake_minimum_required(VERSION 2.8.3)
project(baker_wet_cleaning_application)

set(catkin_RUN_PACKAGES
	actionlib
	actionlib_msgs
	baker_msgs
	cob_srvs
	cv_bridge
	dynamic_reconfigure
	eigen_conversions
	geometry_msgs
	ipa_building_msgs
	move_base_msgs
	nav_msgs
	roscpp
	rospy
	scitos_msgs
	sensor_msgs
	std_msgs
	std_srvs
	tf
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Add the Files for actionlib
add_action_files(
	DIRECTORY action
	FILES 
		InterruptAction.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		actionlib_msgs
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
INCLUDE_DIRS
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	OpenCV
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

# map management client
add_executable(map_management_client
	src/map_management_client.cpp
)
target_link_libraries(map_management_client
	${catkin_LIBRARIES} 
	${OpenCV_LIBS}
)
add_dependencies(map_management_client 
	${catkin_EXPORTED_TARGETS}
#	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
# 	scripts/my_python_script
# 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#	 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#	 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#	 FILES_MATCHING PATTERN "*.h"
#	 PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#	 # myfile1
#	 # myfile2
#	 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
