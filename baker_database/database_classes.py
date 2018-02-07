"""
Assumed procedure of cleaning:

1. Check if a scheduled assignment is to be done
2. Get all rooms to clean
	2.1. Get all the rooms which were incompleted the last time
	2.2. Get all rooms where the last successful cleanup is too far in the past
	2.3. Get all rooms where a defined date for cleanup is given and matches with today.
	2.4. Check if aquired rooms have issues which are not yet resolved. Handle them in some way.
3. Make sequence of the rooms
4. Handle the rooms one by one
	4.1. Go to next room, explore it, clean it
	4.2. Check result
		4.2.1. If there are issues with the cleaning
			4.2.1a. If the issue can be resolved in some easy way, resolve it. Continue as if room is fine.
			4.2.1b. Otherwise, add an issue item to the room
		4.2.2. If there are no issues (maybe also otherwise?), update the date property of the room
	4.3. Redo 4 until all rooms are completed
5. Handle the error items of the rooms
	5.1. Consult human
	5.2. Do some action
		5.2a. If human says what to do (e.g. also to discard), do what the human says
		5.2b. Otherwise, do what the human says
	5.3. Check if issue is resolved
		5.3a. If issue resolved, remove issue item from the room and delete all associated files
		5.3b. Otherwise, keep issue item or tag it as unsolvable
	5.4. Redo from 5.2 on until no handled issues exist

X. If interrupted, store all data to the database
X. After interruption, continue with 2 if there are rooms which were not completed. Else, continue with 5.
"""

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
	# Name of the room for user 
	# (STRING)
	room_name_ = ""
	# Room ID 
	# (INTEGER)
	room_id_ = 0
	# Date of the last successful clean 
	# (DATETIME)
	last_successful_clean_date_ = None
	# Was the last scheduled cleanup interrupted / cancelled and therefore should it be resumed? 
	# (BOOLEAN)
	last_cleanup_unsuccessful_ = False
	# List of issues in a room. Array of RoomIssue 
	# (ARRAY OF ROOMISSUE)
	room_issues_ = []
	# Map of the room 
	# (STRING)
	room_map_ = None
	# Coords of the room center 
	# (POINT32)
	room_center_coords_ = None

# Item that contains assignment data
class AssignmentItem():
	# Name of schedule for user 
	# (STRING)
	assignment_name_ = ""
	# Assignment ID 
	# (INTEGER)
	assignment_id_ = 0
	# Rooms which are handled by this assignment 
	# (ARRAY OF ROOMITEM)
	scheduled_rooms_ = []
	# Date of the last clean which completed successfully 
	# (DATETIME)
	last_completed_clean_ = None
	# Interval in which the rooms are supposed to be cleaned 
	# (TIMEDELTA)
	clean_interval_ = None
	# List of dates where the assignment should be done anyway 
	# (ARRAY OF DATETIME)
	clean_dates_ = []

	# Options
	# Should interrupted assignments be completed automatically after interruption is over? 
	# (BOOLEAN)
	shall_auto_complete_ = True
