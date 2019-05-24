cmake_minimum_required(VERSION 2.8.3)
project(ipa_dirt_detection)

set(catkin_RUN_PACKAGES
	cv_bridge
	dynamic_reconfigure
	geometry_msgs
	image_transport
	pcl_ros
	roscpp
	roslib
	sensor_msgs
	std_msgs
	std_srvs
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
)

find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PCL 1.3 REQUIRED)


# dynamic reconfigure
generate_dynamic_reconfigure_options(
	cfg/DirtDetection.cfg
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
		common/include
		ros/include
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		OpenCV
		Boost
)

###########
## Build ##
###########
include_directories(
	common/include
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

### dirt_detection
add_executable(dirt_detection
	ros/src/dirt_detection.cpp
)
target_link_libraries(dirt_detection
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
	${PCL_INCLUDE_DIRS}
)
add_dependencies(dirt_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


### dirt_detection_client
add_executable(dirt_detection_client
	ros/src/dirt_detection_client.cpp
)
target_link_libraries(dirt_detection_client
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(dirt_detection_client ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS dirt_detection dirt_detection_client
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY common/files
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
	PATTERN ".svn" EXCLUDE
)
