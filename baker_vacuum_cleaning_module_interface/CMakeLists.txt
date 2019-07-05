cmake_minimum_required(VERSION 2.8.3)
project(baker_vacuum_cleaning_module_interface)

set(catkin_RUN_PACKAGES
	baker_msgs
	roscpp
	std_srvs
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

#find_package(OpenCV REQUIRED)

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
	#OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# include cmake file for MIRA
include(CMakeLists.mira)


# vacuum_cleaning_module_interface
add_executable(vacuum_cleaning_module_interface
	src/VacuumCleaningModuleInterface.cpp
)
target_link_libraries(vacuum_cleaning_module_interface
	${catkin_LIBRARIES} 
	#${OpenCV_LIBS}
)
add_dependencies(vacuum_cleaning_module_interface
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
install(TARGETS vacuum_cleaning_module_interface
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

## Mark launch files for installation
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
	FILES_MATCHING PATTERN "*.launch"
)

## Mark resources files for installation
install(DIRECTORY etc/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
	FILES_MATCHING PATTERN "*"
)

##########
## MIRA ##
##########

# for mira support (auto linking, enabling C++0x support,...)
catkin_add_mira(vacuum_cleaning_module_interface)
