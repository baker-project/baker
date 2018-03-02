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
	# (TIMEDELTA)
	max_aux_time_ = None
	# Amount of days between two executions of the same assignment. Default is two weeks (14)
	# (INTEGER)
	assignment_timedelta_ = 14
	







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

	# DATA AQUIRED FROM THE DATABASE
	# ==============================

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

	# DATA CALCULATED OR AQUIRED FROM OTHER SOURCES
	# =============================================

	# Date of the last successful clean 
	# (DATETIME)
	last_successful_clean_date_ = None
	# Was the last scheduled cleanup interrupted / cancelled and therefore should it be resumed? 
	# (BOOLEAN)
	last_cleanup_successful_ = False
	# List of issues in a room. Array of RoomIssue 
	# (ARRAY OF ROOMISSUE)
	room_issues_ = []
	# Filename of the room map file
	# (STRING)
	room_map_ = None
	# CV_Bridge representation of the map
	# (CV_BRIDGE)
	room_map_data_ = None
	# Coords of the room center 
	# (POINT32)
	room_center_coords_ = None

	# MISCELLANEOUS STUFF
	# ===================

	# RoomItems must be hashable for convenience
	def __hash__(self):
		return self.room_id_






# Item that contains assignment data
class AssignmentItem():

	# DATA AQUIRED FROM THE DATABASE
	# ==============================

	# Name of schedule for user 
	# (STRING)
	assignment_name_ = ""
	# Week type [0=Even, 1=Uneven]
	# (INTEGER)
	assignment_week_type_ = 0
	# Day type [0=Monday, ..., 6=Sunday]
	# (INTEGER)
	assignment_week_day_ = 0
	# IDs of rooms which are to be cleaned completely. This uses RoomItem.room_id_!!
	# (ARRAY OF INTEGER)
	scheduled_rooms_cleaning_ = []
	# IDs of rooms where only the trashcan has to be emptied. This uses RoomItem.room_id_!!
	# (ARRAY OF INTEGER)
	scheduled_rooms_trashcan_ = []
	

	# DATA CALCULATED OR AQUIRED FROM OTHER SOURCES
	# =============================================

	# The RoomItem objects behind the IDs for cleaning
	# (ARRAY OF RoomItem)
	scheduled_rooms_cleaning_data_ = []
	# The RoomItem objects behind the IDs for trashcan
	# (ARRAY OF RoomItem)
	scheduled_rooms_trashcan_data_ = []
	# Date of the last clean which completed successfully 
	# (DATETIME)
	last_completed_clean_ = None

