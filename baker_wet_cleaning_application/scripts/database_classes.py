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



# Class which contains all application-wide data to be recorded
class GlobalApplicationData():
	# Last date the application was run
	# (DATE)
	last_execution_date_ = None
	# Was the last saving of the database successful?
	# (BOOLEAN)
	last_database_save_successful_ = True



# Class which resembles a log item, documenting a specific event
class LogBaseItem():
	# Week and day flag [week, day]
	# ([INTEGER, INTEGER])
	log_week_and_day_ = [0, 0]
	# Datetime stating the time of completion
	# (DATETIME)
	log_datetime_ = None



# Class which resembles a log for a room specific event
class RoomLogItem(LogBaseItem):
	# Room ID
	# (INTEGER)
	log_room_id_ = 0
	# ID of the cleaning task which was completed
	# (INTEGER)
	log_cleaning_type_ = 0
	# Amount of trashcans which were found
	# (INTEGER)
	log_trashcan_count_ = 0
	# Surface area which has been effectively cleaned
	# (FLOAT)
	log_cleaned_area_ = 0
	# Status of the task completion [Successful, Erroneous, ...]
	# (INTEGER)
	log_cleaning_status_ = 0
	# Dirt spots which have been detected
	# (?)
	log_detected_dirt_spots_ = []
	# Issues in the room
	# (ARRAY OF ROOMISSUE)
	log_room_issues_ = []
	# Battery usage
	# (INTEGER)
	log_battery_usage_ = 0
	# Water usage
	# (FLOAT)
	log_water_usage_ = 0
	# Trolley load
	# (INTEGER)
	log_trolley_load_ = 0



# Class which contains all log items
class Log():
	# List of all room log items which belong to a room
	# (DICTIONARY OF ARRAY OF ROOMLOGITEM)
	room_logs_ = {}
	# Log item for machine events
	# (ROBOTLOGITEM)
	robot_log_ = None



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

