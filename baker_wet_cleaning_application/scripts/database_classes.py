#!/usr/bin/env python

# ========================================================================
# Description:
# Contains all classes whose instances are stored in the database
# ========================================================================


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

	def __str__(self):
		output = "[RobotProperties]:\n"
		output += "\tRadius {} | coverage radius {} | fov {}\n".format(self.exploration_robot_radius_,
																		self.exploration_coverage_radius_, self.exploration_field_of_view_)
		output += "\tFrameId {}\n".format(self.exploration_header_frame_id_)
		output += "\tPath tolerance {} | Goal tolerance {} | Angle tolerance {}\n".format(self.path_follow_path_tolerance_,
																						  self.path_follow_goal_position_tolerance_, self.path_follow_goal_angle_tolerance_)
		output += "\tWall tolerance {} | Wall position tolerance {} | Wall angle tolerance {}\n".format(self.wall_follow_path_tolerance_,
																										self.wall_follow_goal_position_tolerance_,
																										self.wall_follow_goal_angle_tolerance_)
		return output

# Class which contains all global application settings
class GlobalSettings():
	# Should incomplete assignments be completed in the next opportunity?
	# (BOOLEAN)
	shall_auto_complete_ = True  # todo (rmb-ma). Unused. Remove ?
	# Maximum time the robot has for completing missed cleanups
	# (FLOAT)
	max_aux_time_ = 0  # todo (rmb-ma). Unused. Remove ?
	# Amount of days between two executions of the same assignment. Default is two weeks (14)
	# (INTEGER)
	assignment_timedelta_ = 14  # todo (rmb-ma). It's harcoded in some places (in the getTodayIndex functions for example)
	# should be a number of weeks?

	def __str__(self):
		return "[GlobalSettings]: autocomplete {} | maxAuxTime {} | timeDelta {}".format(self.shall_auto_complete_,
																						 self.max_aux_time_,
																						 self.assignment_timedelta_)

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
	# Segmented map image file
	# (CVBRIDGE)
	map_image_segmented_ = None
	# Map header frame id
	# (STRING)
	map_header_frame_id_ = ""


# Class which contains all application-wide data to be recorded
class GlobalApplicationData():

	PROGRESS_COMPLETED = 0
	PROGRESS_RUNNING = 1
	PROGRESS_PAUSED = 2
	PROGRESS_STOPPED = 3
	PROGRESS_DISCARDED = 4

	# Last date the application was started
	# (DATETIME)
	last_execution_date_ = None
	# Dates when the date planning subtasks of application_wet_cleaning completed the last time [Due cleaning, Overdue cleaning]
	# ([DATETIME, DATETIME])
	last_planning_date_ = [None, None]
	# Application run count of that day
	# (INTEGER)
	run_count_ = 0
	# Was the last saving of the database successful?
	# (BOOLEAN)
	last_database_save_successful_ = True
	# Progress status of the application [0=Completed, 1=Running, 2=Paused, 3=Stopped, 4=Discarded] and date the progress belongs to
	# ([INTEGER, DATETIME])
	progress_ = [PROGRESS_COMPLETED, None]

	# planning time offset for setting zero time of the day, in [minutes]
	planning_offset_ = 0



# Class which resembles a log item, documenting a specific event
class LogItem():

	# Week and day flag [week, day]
	# ([INTEGER, INTEGER])
	log_week_and_day_ = [0, 0]
	# Datetime stating the time of the concerning event
	# (DATETIME)
	date_and_time_ = None
	# Concerning room id 
	# (INTEGER)
	room_id_ = 0
	# Concerning cleaning task [-1=trashcan_only, 0=dry_only, 1=wet_only]
	# (INTEGER)
	cleaning_task_ = 0
	# Status [Started, Completed=1, Stopped, Halted, Paused, Continued, ...]
	# (INTEGER)
	status_ = 0
	# Found trashcans
	# (INTEGER)
	found_trashcans_ = 0
	# Found dirt spots
	# (INTEGER)
	found_dirtspots_ = 0
	# Cleaned floor surface area
	# (INTEGER)
	cleaned_surface_area_ = 0
	# Amount of used water
	# (FLOAT)
	used_water_amount_ = 0
	# Trolley capacity
	# (FLOAT)
	trolley_capacity_ = 0
	# Battery usage
	# (FLOAT)
	battery_usage_ = 0
	# IDs of occurred room issues
	# (ARRAY OF INTEGER)
	room_issues_ = []



# Class that describes all information on an issue
class RoomIssue():
	# Issue ID 
	# (INTEGER)
	issue_id_ = 0
	# Room ID of the RoomItem the issue belongs to
	# (INTEGER)
	room_id_ = 0
	# Type of issue [0=?, ..., n=?] 
	# (INTEGER)
	issue_type_ = 0
	# File names of the pictures "<NameOfRoom>_<Date>_<IssueNumber>_<Number>.<jpg or whatever>" 
	# (ARRAY OF STRING)
	issue_images_ = [] # todo rmb-ma check
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
	# Room surface type [0=?, ..., n=?]
	# (INTEGER)
	room_surface_type_ = 0
	# Room cleaning method [0=dry, 1=wet, 2=both]
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

	# DATA CALCULATED OR ACQUIRED FROM OTHER SOURCES
	# =============================================

	# List of all last successful cleaning dates. [Trashcan, Dry, Wet]
	# (ARRAY OF DATETIME)
	room_cleaning_datestamps_ = [None, None, None]
	# List of issues in a room. Array of RoomIssue 
	# (ARRAY OF ROOMISSUE)
	room_issues_ = []
	# Filename of the room map file
	# (STRING)
	room_map_filename_ = ""
	# CV_Bridge representation of the map
	# (CV_BRIDGE)
	room_map_data_ = None
	# Room Information in pixel
	# (ROOMINFORMATION)
	room_information_in_pixel_ = None
	# Room information in meter
	# (ROOMINFORMATION)
	room_information_in_meter_ = None
	# The cleaning tasks which currently are to be performed. [-1=trashcan_only, 0=dry_only, 1=wet_only]
	# (ARRAY OF INTEGER)
	open_cleaning_tasks_ = [] 

	# MISCELLANEOUS STUFF
	# ===================

	# RoomItems must be hashable for convenience
	def __hash__(self):
		return self.room_id_

	def __str__(self):
		return "[Room {}]".format(self.room_id_)
