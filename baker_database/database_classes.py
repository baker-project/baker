#!/usr/bin/env python


# Class which contains all the properties of the robot
class RobotProperties():
	# Exploration server constants
	exploration_robot_radius_ = 0
	exploration_coverage_radius_ = 0
	exploration_field_of_view_ = None
	# Move_Base server constants
	exploration_header_frame_id_ = ""
	# Path following server constants
	path_follow_path_tolerance_ = 0
	path_follow_goal_position_tolerance_ = 0
	path_follow_goal_angle_tolerance_ = 0
	# Wall following server constants
	wall_follow_path_tolerance_ = 0
	wall_follow_goal_position_tolerance_ = 0
	wall_follow_goal_angle_tolerance_ = 0





# Class which contains all global application settings
class GlobalSettings():
	# Should incomplete assignments be completed in the next opportunity?
	# (BOOLEAN)
	shall_auto_complete_ = True
	# Maximum time the robot has for completing missed cleanups
	# (FLOAT)
	max_aux_time_ = 0
	# Amount of days between two executions of the same assignment. Default is two weeks (14)
	# (INTEGER)
	assignment_timedelta_ = 14
	



# Class which contains all global map data
class GlobalMapData():
	# Map resolution
	# (FLOAT32)
	map_resolution_ = 0
	# Map origin
	# (ARRAY OF FLOAT)
	map_origin_ = []
	# Map image file
	# (CVBRIDGE)
	map_image_ = None
	# Map header frame id
	# (STRING)
	map_header_frame_id_ = ""






# Class that describes all information on an issue
class RoomIssue():
	# Issue ID 
	# (INTEGER)
	issue_id_ = 0
	# Type of issue [0=?, ..., n=?] 
	# (INTEGER)
	issue_type_ = None
	# File names of the pictures "<NameOfRoom>_<Date>_<IssueNumber>_<Number>.<jpg or whatever>" 
	# (ARRAY OF STRING)
	issue_images_ = []
	# Issue coordinates 
	# (POINT32)
	issue_coords_ = None
	# Date the issue was discovered 
	# (DATETIME)
	issue_date_ = None






# Item that contains information on a room
class RoomItem():

	# DATA AQUIRED FROM THE ROOM AND TERRITORY PLAN
	# =============================================

	# Name of the room for user 
	# (STRING)
	room_name_ = ""
	# Room ID 
	# (INTEGER)
	room_id_ = 0
	# Position ID (eg "4.26")
	# (STRING)
	room_position_id_ = ""
	# Floor (e.g. "OG1")
	# (STRING)
	room_floor_id_ = ""
	# Building (e.g. "Hauptgebaeude")
	# (STRING)
	room_building_id_ = ""
	# Territory the room belongs to
	# (STRING)
	room_territory_id_ = ""
	# Room surfcae type [0=?, ..., n=?]
	# (INTEGER)
	room_surface_type_ = 0
	# Room cleaning method [0=?, ..., n=?]
	# (INTEGER)
	room_cleaning_method_ = 0
	# Room surface area
	# (FLOAT)
	room_surface_area_ = 0.0
	# Amount of trashcans in the room
	# (INTEGER)
	room_trashcan_count_ = 0
	# Days the room shall be cleaned. (Item position = day, Item = cleaning type)
	# (ARRAY OF STRING)
	room_scheduled_days_ = []

	# DATA CALCULATED OR AQUIRED FROM OTHER SOURCES
	# =============================================

	# List of all last successful cleaning dates. [Trashcan, Dry, Wet]
	# (ARRAY OF DATETIME)
	room_cleaning_datestamps_ = [None, None, None]
	# List of issues in a room. Array of RoomIssue 
	# (ARRAY OF ROOMISSUE)
	room_issues_ = []
	# Filename of the room map file
	# (STRING)
	room_map_ = ""
	# CV_Bridge representation of the map
	# (CV_BRIDGE)
	room_map_data_ = None
	# Room Information in pixel
	# (ROOMINFORMATION)
	room_information_in_pixel_ = None
	# Room information in meter
	# (ROOMINFORMATION)
	room_information_in_meter_ = None

	# TODO: REMOVE
	# Date of the last successful cleaning
	# (DATETIME)
	last_successful_cleaning_date_ = None
	# Date of the last successful trashcan emptying
	# (DATETIME)
	last_successful_trashcan_date_ = None

	# MISCELLANEOUS STUFF
	# ===================

	# RoomItems must be hashable for convenience
	def __hash__(self):
		return self.room_id_

