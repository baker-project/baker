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
	# Room surfcae type [0=?, ..., n=?]
	# (INTEGER)
	room_surface_type_ = 0
	# Room surface area
	# (FLOAT)
	room_surface_area_ = 0.0
	# Amount of trashcans in the room %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NYI
	# (INTEGER)
	room_trashcan_count_ = 0

	# DATA AQUIRED FROM OTHER SOURCES
	# ===============================

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
	# Assignment ID 
	# (INTEGER)
	assignment_id_ = 0
	# IDs of Rooms which are handled by this assignment 
	# (ARRAY OF INTEGER)
	scheduled_rooms_ = []
	# Interval in which the rooms are supposed to be cleaned 
	# (TIMEDELTA)
	clean_interval_ = None

	# DATA AQUIRED FROM OTHER SOURCES
	# ===============================

	# The RoomItem objects behind the IDs
	# RoomItem
	scheduled_rooms_data_ = []
	# Date of the last clean which completed successfully 
	# (DATETIME)
	last_completed_clean_ = None

	# Settings which only concern this assignment
	# (none so far)
