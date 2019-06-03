#!/usr/bin/env python

# For the room and robot data information
import database_classes
# For time calculations
from datetime import datetime, date, time, timedelta
# For Point32
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
# For map receiving, for image receiving
import cv2
from cv_bridge import CvBridge, CvBridgeError
# For support of the JSON format
import json
# For copying, finding and deleting JSON files
from shutil import copyfile
import os
# For room information
from ipa_building_msgs.msg import RoomInformation

# Database class
class Database:

	#========================================================================
	# Description:
	# Container for all data being used in the cleaning application.
	# Also contains functions to load and save data.
	#========================================================================

	# Rooms
	rooms_ = []
	# Global map data
	global_map_data_ = None
	# Robot properties
	robot_properties_ = None
	# Global settings
	global_settings_ = None
	# Application data
	application_data_ = None
	# File names
	rooms_filename_ = ""
	tmp_rooms_filename_ = ""
	application_data_filename_ = ""
	tmp_application_data_filename_ = ""
	log_filepath_ = ""
	tmp_log_filepath_ = ""
	global_settings_filename_ = ""
	robot_properties_filename_ = ""
	global_map_data_filename_ = ""
	global_map_image_filename_ = ""
	global_map_segmented_image_filename_ = ""


# =========================================================================================
# Private methods
# =========================================================================================

	@staticmethod
	def datetimeToString(datetime_date):
		return datetime_date.strftime("%Y-%m-%d_%H:%M")


	@staticmethod
	def stringToDatetime(string_date):
		return datetime.strptime(string_date, "%Y-%m-%d_%H:%M")


	@staticmethod
	def point32ToArray(point32_point):
		return [point32_point.x, point32_point.y, point32_point.z]


	@staticmethod
	def arrayToPoint32(array_point):
		return Point32(x=array_point[0], y=array_point[1], z=array_point[2])


	def updateGlobalApplicationData(self, dict):
		self.application_data_ = database_classes.GlobalApplicationData()
		# Get the datetime of the last recorded start of the application
		last_execution_date_str = dict.get("last_execution_date")
		if last_execution_date_str is not None:
			self.application_data_.last_execution_date_ = self.stringToDatetime(last_execution_date_str)
		else:
			self.application_data_.last_execution_date_ = None
		# Get the last planning dates (see database_classes.py for further explanation)
		last_planning_date_str = dict.get("last_planning_date")
		if (last_planning_date_str[0] is not None) and (last_planning_date_str[1] is not None):
			self.application_data_.last_planning_date_ = [self.stringToDatetime(last_planning_date_str[0]), self.stringToDatetime(last_planning_date_str[1])]
		else:
			self.application_data_.last_planning_date_ = [None, None]
		# Get if the last database saving routine completed without error
		self.application_data_.last_database_save_successful_ = dict.get("last_database_save_successful")
		# Get the amount of executions of the application on the current day. Increment by one.
		self.application_data_.run_count_ = dict.get("run_count")
		# Get the progress variable
		progress_str = dict.get("progress")
		self.application_data_.progress_[0] = progress_str[0]
		if progress_str[1] is not None:
			self.application_data_.progress_[1] = self.stringToDatetime(progress_str[1])
		else:
			self.application_data_.progress_[1] = None


	def getGlobalApplicationDataDictFromGlobalApplicationData(self):
		application_data_dict = {}
		
		execution_date_datetime = self.application_data_.last_execution_date_
		if execution_date_datetime is not None:
			application_data_dict["last_execution_date"] = self.datetimeToString(execution_date_datetime)
		else:
			application_data_dict["last_execution_date"] = None
		
		planning_date_datetime = [None, None]
		if self.application_data_.last_planning_date_[0] is not None:
			planning_date_datetime[0] = self.datetimeToString(self.application_data_.last_planning_date_[0])
		if self.application_data_.last_planning_date_[1] is not None:
			planning_date_datetime[1] = self.datetimeToString(self.application_data_.last_planning_date_[1])
		application_data_dict["last_planning_date"] = planning_date_datetime
		
		application_data_dict["last_database_save_successful"] = self.application_data_.last_database_save_successful_
		application_data_dict["run_count"] = self.application_data_.run_count_

		if self.application_data_.progress_[1] is not None:
			application_data_dict["progress"] = [self.application_data_.progress_[0], self.datetimeToString(self.application_data_.progress_[1])]
		else:
			application_data_dict["progress"] = [self.application_data_.progress_[0], None]
		
		return application_data_dict
	

	def updateGlobalSettings(self, dict_settings):
		self.global_settings_ = database_classes.GlobalSettings()
		self.global_settings_.shall_auto_complete_ = dict_settings.get("shall_auto_complete")
		self.global_settings_.max_aux_time_ = dict_settings.get("max_aux_time")
		self.global_settings_.assignment_timedelta_ = dict_settings.get("assignment_timedelta")


	def getGlobalSettingsDictFromGlobalSettings(self):
		global_settings_dict = {}
		global_settings_dict["shall_auto_complete"] = self.global_settings_.shall_auto_complete_
		global_settings_dict["max_aux_time"] = self.global_settings_.max_aux_time_
		global_settings_dict["assignment_timedelta"] = self.global_settings_.assignment_timedelta_
		return global_settings_dict


	def updateGlobalMapData(self, dict):
		bridge = CvBridge()
		self.global_map_data_ = database_classes.GlobalMapData()
		# Get an open cv representation of the map
		map_opencv = cv2.imread(self.global_map_image_filename_, 0)
		self.global_map_data_.map_image_ = bridge.cv2_to_imgmsg(map_opencv, encoding = "mono8")
		# Get an open cv representation of the segmented map
		map_segmented_opencv = cv2.imread(self.global_map_segmented_image_filename_, 0)
		self.global_map_data_.map_image_segmented_ = bridge.cv2_to_imgmsg(map_segmented_opencv, encoding = "mono8") 
		# Get the map resolution
		self.global_map_data_.map_resolution_ = dict.get("map_resolution")
		# Get the map origin
		map_origin_coords = dict.get("map_origin")
		pose = Pose()
		pose.position.x = map_origin_coords[0]
		pose.position.y = map_origin_coords[1]
		pose.position.z = map_origin_coords[2]
		pose.orientation.w = map_origin_coords[3]
		pose.orientation.x = map_origin_coords[4]
		pose.orientation.y = map_origin_coords[5]
		pose.orientation.z = map_origin_coords[6]
		self.global_map_data_.map_origin_ = pose
		# Get the map header frame id
		self.global_map_data_.map_header_frame_id_ = dict.get("map_header_frame_id")


	def updateRobotProperties(self, dict):
		self.robot_properties_ = database_classes.RobotProperties()
		# Exploration server constants
		self.robot_properties_.exploration_robot_radius_ = dict.get("exploration_robot_radius")
		self.robot_properties_.exploration_coverage_radius_ = dict.get("exploration_coverage_radius")
		e_fov_list = dict.get("exploration_field_of_view")  # todo: only supports 4 point FOVs
		exploration_field_of_view = [Point32(x=e_fov_list[0][0], y=e_fov_list[0][1]), Point32(x=e_fov_list[1][0], y=e_fov_list[1][1]), Point32(x=e_fov_list[2][0], y=e_fov_list[2][1]), Point32(x=e_fov_list[3][0], y=e_fov_list[3][1])]
		self.robot_properties_.exploration_field_of_view_ = exploration_field_of_view
		# Move_Base server constants
		self.robot_properties_.exploration_header_frame_id_ = dict.get("exploration_header_frame_id")
		# Path following server constants
		self.robot_properties_.path_follow_path_tolerance_ = dict.get("path_follow_path_tolerance")
		self.robot_properties_.path_follow_goal_position_tolerance_ = dict.get("path_follow_goal_position_tolerance")
		self.robot_properties_.path_follow_goal_angle_tolerance_ = dict.get("path_follow_goal_angle_tolerance")
		# Wall following server constants
		self.robot_properties_.wall_follow_path_tolerance_ = dict.get("wall_follow_path_tolerance")
		self.robot_properties_.wall_follow_goal_position_tolerance_ = dict.get("wall_follow_goal_position_tolerance")
		self.robot_properties_.wall_follow_goal_angle_tolerance_ = dict.get("wall_follow_goal_angle_tolerance")


	# Make rooms_ contain all the rooms stated in the dict parameter
	def updateRoomsList(self, dict_settings):
		self.rooms_ = []
		for room_key in dict_settings:
			room_issues = []
			current_room = database_classes.RoomItem()
			# Get all issues of a room and get all properties of those issues
			issues_dict = dict_settings.get(room_key).get("room_issues")
			for issue_key in issues_dict:
				current_issue = database_classes.RoomIssue()
				# Get the room issue ID
				current_issue.issue_id_ = issues_dict.get(issue_key).get("issue_id")
				# Get the room issue type
				current_issue.issue_type_ = issues_dict.get(issue_key).get("issue_type")
				# Get the room issue images
				current_issue.issue_images_ = issues_dict.get(issue_key).get("issue_images")
				# Get the room issue coordinates
				issue_coords_list = issues_dict.get(issue_key).get("issue_coords")
				current_issue.issue_coords_ = Point32(x=issue_coords_list[0], y=issue_coords_list[1], z=issue_coords_list[2])
				# Get the date the issue was detected
				current_issue.issue_date_ = self.stringToDatetime(issues_dict.get(issue_key).get("issue_date"))
				# Append current room issue to the room_issues list
				room_issues.append(current_issue)
			current_room.room_issues_ = room_issues
			# Get the name of the room
			current_room.room_name_ = dict_settings.get(room_key).get("room_name")
			# Get the ID of the room
			current_room.room_id_ = dict_settings.get(room_key).get("room_id")
			# Get the position ID of the room
			current_room.room_position_id_ = dict_settings.get(room_key).get("room_position_id")
			# Get the floor of the room
			current_room.room_floor_id_ = dict_settings.get(room_key).get("room_floor_id")
			# Get the building ID of the room
			current_room.room_building_id_ = dict_settings.get(room_key).get("room_building_id")
			# Get the territory the room is in
			current_room.room_territory_id_ = dict_settings.get(room_key).get("room_territory_id")
			# Get the map of the room
			current_room.room_map_filename_ = dict_settings.get(room_key).get("room_map_filename")
			# Get an open cv representation of the map or None if there is no map
			if current_room.room_map_filename_ is not None:
				room_map_file_path = str(self.extracted_file_path) + str("/maps/") + str(current_room.room_map_filename_)
				map_opencv = cv2.imread(room_map_file_path, 0)
				bridge = CvBridge()
				current_room.room_map_data_ = bridge.cv2_to_imgmsg(map_opencv, encoding = "mono8")
			else:
				current_room.room_map_data = None
			# Get the room information
			pixel_coords = dict_settings.get(room_key).get("room_information_in_pixel")
			current_room.room_information_in_pixel_ = RoomInformation()
			current_room.room_information_in_pixel_.room_center = Point32(x=pixel_coords[0][0], y=pixel_coords[0][1], z=pixel_coords[0][2])
			current_room.room_information_in_pixel_.room_min_max.points.append(Point32(x=pixel_coords[1][0], y=pixel_coords[1][1], z=pixel_coords[1][2]))
			current_room.room_information_in_pixel_.room_min_max.points.append(Point32(x=pixel_coords[2][0], y=pixel_coords[2][1], z=pixel_coords[2][2]))
			meter_coords = dict_settings.get(room_key).get("room_information_in_meter")
			current_room.room_information_in_meter_ = RoomInformation()
			current_room.room_information_in_meter_.room_center = Point32(x=meter_coords[0][0], y=meter_coords[0][1], z=meter_coords[0][2])
			current_room.room_information_in_meter_.room_min_max.points.append(Point32(x=meter_coords[1][0], y=meter_coords[1][1], z=meter_coords[1][2]))
			current_room.room_information_in_meter_.room_min_max.points.append(Point32(x=meter_coords[2][0], y=meter_coords[2][1], z=meter_coords[2][2]))
			# Get the room surface type
			current_room.room_surface_type_ = dict_settings.get(room_key).get("room_surface_type")
			# Get the cleaning method of the room
			current_room.room_cleaning_method_ = dict_settings.get(room_key).get("room_cleaning_method")
			# Get the room surface area
			current_room.room_surface_area_ = dict_settings.get(room_key).get("room_surface_area")
			# Get the room trashcan count
			current_room.room_trashcan_count_ = dict_settings.get(room_key).get("room_trashcan_count")
			# Get the days where the room has to be cleaned in a specified way
			current_room.room_scheduled_days_ = dict_settings.get(room_key).get("room_scheduled_days")
			# Get the yet open cleaning tasks
			current_room.open_cleaning_tasks_ = dict_settings.get(room_key).get("open_cleaning_tasks")
			
			# Get the list with the datestamps
			string_datestamp_list = dict_settings.get(room_key).get("room_cleaning_datestamps")
			datestamps = []
			for datestamp in string_datestamp_list:
				if (datestamp is not None):
					datestamps.append(self.stringToDatetime(datestamp))
				else:
					datestamps.append(None)
			current_room.room_cleaning_datestamps_ = datestamps
			
			# Append current room object to the rooms_ list
			self.rooms_.append(current_room)


	# Get a dictionary representation of rooms_
	def getRoomsDictFromRoomsList(self):
		room_dict = {}
		for current_room in self.rooms_:
			# Check if current_room is a room 
			if isinstance(current_room, database_classes.RoomItem) :
				# Make a dict of the issues of current_room
				issues_dict = {}
				for current_issue in current_room.room_issues_:
					# Check if current_issue is an issue
					if isinstance(current_issue, database_classes.RoomIssue):
						# Fill in a string representation of the date
						date_str_issue = current_issue.issue_date_.strftime("%Y-%m-%d_%H:%M")
						# Fill in the issue coordinates
						ic_x = current_issue.issue_coords_.x
						ic_y = current_issue.issue_coords_.y
						ic_z = current_issue.issue_coords_.z
						issue_coords_list = [ic_x, ic_y, ic_z]
						# Fill in the dictionary with the data
						issues_dict[str(current_issue.issue_id_)] = {
							"issue_id": current_issue.issue_id_,
							"room_id": current_issue.room_id_,
							"issue_type": current_issue.issue_type_,
							"issue_images": current_issue.issue_images_,
							"issue_coords": issue_coords_list,
							"issue_date": date_str_issue
						}
					else:
						print "[FATAL]: An element in issues array is not an issue object!"
				
				
				# Fill in the datestamps
				datestamp_list = []
				for datestamp in current_room.room_cleaning_datestamps_:
					if datestamp is not None:
						datestamp_list.append(self.datetimeToString(datestamp))
					else:
						datestamp_list.append(None)
				# Fill in the room information
				if (current_room.room_information_in_meter_ is not None) and (current_room.room_information_in_pixel_ is not None):
					px_center_list = self.point32ToArray(current_room.room_information_in_pixel_.room_center)
					px_min_list = self.point32ToArray(current_room.room_information_in_pixel_.room_min_max.points[0])
					px_max_list = self.point32ToArray(current_room.room_information_in_pixel_.room_min_max.points[1])
					room_information_in_pixel_list = [px_center_list, px_min_list, px_max_list]
					meter_center_list = self.point32ToArray(current_room.room_information_in_meter_.room_center)
					meter_min_list = self.point32ToArray(current_room.room_information_in_meter_.room_min_max.points[0])
					meter_max_list = self.point32ToArray(current_room.room_information_in_meter_.room_min_max.points[1])
					room_information_in_meter_list = [meter_center_list, meter_min_list, meter_max_list]
				else:
					room_information_in_pixel_list = [None, None, None]
					room_information_in_meter_list = [None, None, None]
				# Fill the dictionary with the data
				room_dict[str(current_room.room_id_)] = {
					"room_id": current_room.room_id_,
					"room_name": current_room.room_name_,
					"room_position_id": current_room.room_position_id_,
					"room_floor_id": current_room.room_floor_id_,
					"room_building_id": current_room.room_building_id_,
					"room_territory_id": current_room.room_territory_id_,
					"room_issues": issues_dict,
					"room_map_filename": current_room.room_map_filename_,
					"room_information_in_pixel": room_information_in_pixel_list,
					"room_information_in_meter": room_information_in_meter_list,
					"room_surface_type": current_room.room_surface_type_,
					"room_cleaning_method": current_room.room_cleaning_method_,
					"room_surface_area": current_room.room_surface_area_,
					"room_trashcan_count": current_room.room_trashcan_count_,
					"room_scheduled_days": current_room.room_scheduled_days_,
					"room_cleaning_datestamps": datestamp_list,
					"open_cleaning_tasks": current_room.open_cleaning_tasks_
				}
			else:
				print "[FATAL]: An element in rooms_ array is not a room object!"
		return room_dict


	# Load temporal/original database from file.
	def readFiles(self, temporal):
		# Load the room data
		if temporal:
			datafile = open(self.tmp_rooms_filename_, "r").read()
		else:
			datafile = open(self.rooms_filename_, "r").read()
		rooms_dict = json.loads(datafile)
		self.updateRoomsList(rooms_dict)

		# Load the application data
		if temporal:
			datafile = open(self.tmp_application_data_filename_, "r").read()
		else:
			datafile = open(self.application_data_filename_, "r").read()
		application_data_dict = json.loads(datafile)
		self.updateGlobalApplicationData(application_data_dict)

		# Load the robot properties
		datafile = open(self.robot_properties_filename_, "r").read()
		robot_properties_dict = json.loads(datafile)
		self.updateRobotProperties(robot_properties_dict)

		# Load the global settings
		datafile = open(self.global_settings_filename_, "r").read()
		global_settings_dict = json.loads(datafile)
		self.updateGlobalSettings(global_settings_dict)

		# Load the global map data
		datafile = open(self.global_map_data_filename_, "r").read()
		global_map_data_dict = json.loads(datafile)
		self.updateGlobalMapData(global_map_data_dict)


	# Check the integrity of the specified file, return True on intact files
	def checkIntegrity(self, temporal):
		try:
			if temporal:
				datafile = open(self.tmp_application_data_filename_, "r").read()
			else:
				datafile = open(self.application_data_filename_, "r").read()
			dict_settings = json.loads(datafile)
			return dict_settings.get("last_database_save_successful")
		except:
			return False


	# Returns the amount of runs of a specific day
	# Run count increases, if application is found completed or discarded
	def updateRunCount(self, date):
		if date is not None:
			delta = datetime.now() - date
			if delta.days == 0:
				self.application_data_.run_count_ = self.application_data_.run_count_ + 1
			else:
				self.application_data_.run_count_ = 1
		else:
			self.application_data_.run_count_ = 1
			

	# Determine what the current logfile name is supposed to be
	def getCurrentLogfileName(self):
		# Filename: log_<year>_<week>_<day>_run<run count>.json
		progress_date = self.application_data_.progress_[1]
		week = progress_date.isocalendar()[1]
		year = progress_date.year
		day = progress_date.weekday()
		run_count = self.application_data_.run_count_
		return "log_" + str(year) + "_" + str(week) + "_" + str(day) + "_run" + str(run_count) + ".json"
		

	# Convert log dict into an object array
	def getLogListFromLogDict(self, dict_settings):
		log_item_list = []
		for log_key in dict_settings:
			log_item = database_classes.LogItem()
			log_item.cleaned_surface_area_ = dict_settings.get(log_key).get("cleaned_surface_area")
			log_item.cleaning_task_ = dict_settings.get(log_key).get("cleaning_task")
			log_item.date_and_time_ = self.stringToDatetime(dict_settings.get(log_key).get("date_and_time"))
			log_item.found_dirtspots_ = dict_settings.get(log_key).get("found_dirtspots")
			log_item.found_trashcans_ = dict_settings.get(log_key).get("found_trashcans")
			log_item.log_week_and_day_ = dict_settings.get(log_key).get("week_and_day")
			log_item.room_id_ = dict_settings.get(log_key).get("room_id")
			log_item.status_ = dict_settings.get(log_key).get("status")
			log_item.used_water_amount_ = dict_settings.get(log_key).get("used_water_amount")
			log_item.battery_usage_ = dict_settings.get(log_key).get("battery_usage")
			log_item_list.append(log_item)
		return log_item_list


	# Convert log object array into dict
	def getLogDictFromLogList(self, log_item_list):
		log_dict = {}
		for log_item in log_item_list:
			date_and_time = self.datetimeToString(log_item.date_and_time_)
			log_dict[str(date_and_time)] = {
				"cleaned_surface_area": log_item.cleaned_surface_area_,
				"cleaning_task": log_item.cleaning_task_,
				"date_and_time": date_and_time,
				"found_dirtspots": log_item.found_dirtspots_,
				"found_trashcans": log_item.found_trashcans_,
				"week_and_day": log_item.log_week_and_day_,
				"room_id": log_item.room_id_,
				"status": log_item.status_,
				"used_water_amount": log_item.used_water_amount_,
				"battery_usage": log_item.battery_usage_
			}
		return log_dict


	# Save the room data
	def saveRoomDatabase(self, temporal=True):
		rooms_dict = self.getRoomsDictFromRoomsList()
		rooms_text = json.dumps(rooms_dict, indent=4, sort_keys=True)
		if temporal:
			datafile = open(self.rooms_filename_, "w")
		else:
			datafile = open(self.tmp_rooms_filename_, "w")
		datafile.write(rooms_text)
		datafile.close()


	# Save the application data
	def saveGlobalApplicationData(self, temporal=True):
		application_data_dict = self.getGlobalApplicationDataDictFromGlobalApplicationData()
		application_data_text = json.dumps(application_data_dict, indent=4, sort_keys=True)
		if temporal:
			datafile = open(self.tmp_application_data_filename_, "w")
		else:
			datafile = open(self.application_data_filename_, "w")
		datafile.write(application_data_text)
		datafile.close()


# =========================================================================================
# Public methods
# =========================================================================================

	# Constructor method
	def __init__(self, extracted_file_path='resources'):
		self.extracted_file_path = extracted_file_path
		self.rooms_filename_ = self.extracted_file_path + str("/json/rooms.json")
		self.tmp_rooms_filename_ = self.extracted_file_path + str("/json/tmp_rooms.json")
		self.robot_properties_filename_ = self.extracted_file_path + str("/json/robot_properties.json")
		self.global_settings_filename_ = self.extracted_file_path + str("/json/robot_settings.json")
		self.global_map_data_filename_ = self.extracted_file_path + str("/json/global_map_data.json")
		self.global_map_image_filename_ = self.extracted_file_path + str("/maps/global_map.png")
		self.global_map_segmented_image_filename_ = self.extracted_file_path + str("/maps/global_map_segmented.png")
		self.application_data_filename_ = self.extracted_file_path + str("/json/application_data.json")
		self.tmp_application_data_filename_ = self.extracted_file_path + str("/json/tmp_application_data.json")
		self.log_filepath_ = self.extracted_file_path + str("/logs/")
		self.loadDatabase()

	# Discard temporal database --> All current progress will be forgotten
	def discardTemporalDatabase(self):
		# Save current log as "_discarded_<old logfile name>.txt"
		current_logfile_filename = self.getCurrentLogfileName()
		current_file_name = str(self.log_filepath_) + str(current_logfile_filename)
		if os.path.isfile(current_file_name):
			current_discarded_logfile_name = str(self.log_filepath_) + "_discarded_" + str(current_logfile_filename)
			copyfile(current_file_name, current_discarded_logfile_name)
		# Remove temporal files from disk
		if os.path.isfile(self.tmp_rooms_filename_):
			os.remove(str(self.tmp_rooms_filename_))
		if os.path.isfile(self.tmp_application_data_filename_):
			os.remove(str(self.tmp_application_data_filename_))
		# Reload database from original files
		self.loadDatabase()
		# Mark the discarding in the final file
		self.application_data_.progress_ = [4, datetime.now()]
		self.saveCompleteDatabase(temporal_file=False)


	# Load database data from files
	def loadDatabase(self):
		# Check if there is a temporal representation
		temporal_room_exists = os.path.isfile(self.tmp_rooms_filename_)
		temporal_appdata_exists = os.path.isfile(self.tmp_application_data_filename_)
		temporal_exists = temporal_appdata_exists and temporal_room_exists
		#try:
		if self.checkIntegrity(temporal_exists):
			self.readFiles(temporal_exists)
		else:
			self.readFiles(not temporal_exists)
		#except:
		#	print "[Database] Loading of database failed. No valid original or temporal JSON file set found. Check for damaged data."
		#	exit(1)


	def addLogEntry(self, log_element):
		# Load current log file. If there is not a suiting log file, create one
		current_logfile_filename = self.getCurrentLogfileName()
		current_file_name = str(self.log_filepath_) + str(current_logfile_filename)
		if os.path.isfile(current_file_name):
			datafile = open(current_file_name, "r").read()
			# Translate text to dict and dict to list of LogItem
			log_item_dict = json.loads(datafile)
			log_item_list = self.getLogListFromLogDict(log_item_dict)
		else:
			# Create empty logfile and empty LogItem list
			datafile = open(current_file_name, "w")
			datafile.write("{}")
			datafile.close
			log_item_list = []

		# Append new LogItem instance to the LogItem list
		log_item_list.append(log_element)
		# Translate LogItem list to dict and dict to text
		log_item_dict = self.getLogDictFromLogList(log_item_list)
		log_text = json.dumps(log_item_dict, indent=4, sort_keys=True)
		# Copy current log file and name the copy _backup_<Filename>.json
		current_backup_file_name = str(self.log_filepath_) + "_backup_" + str(current_logfile_filename)
		copyfile(current_file_name, current_backup_file_name)
		# Save dict into current file
		datafile = open(current_file_name, "w")
		datafile.write(log_text)
		datafile.close
		# Remove backup file
		os.remove(current_backup_file_name)


	# Save the complete database safely, remove temporal data on final save
	def saveCompleteDatabase(self, temporal_file=True):
		self.application_data_.last_database_save_successful_ = False
		self.saveGlobalApplicationData(temporal=temporal_file)
		self.saveRoomDatabase(temporal=temporal_file)
		self.application_data_.last_database_save_successful_ = True
		self.saveGlobalApplicationData(temporal=temporal_file)

		if not temporal_file:
			if os.path.isfile(self.tmp_rooms_filename_):
				os.remove(str(self.tmp_rooms_filename_))
			if os.path.isfile(self.tmp_application_data_filename_):
				os.remove(str(self.tmp_application_data_filename_))


	# Retrieve a room by providing a room_id
	def getRoomById(self, room_id):
		result = None
		for i in range(len(self.rooms_)):
			if self.rooms_[i].room_id_ == room_id:
				result = self.rooms_[i]
		return result

"""

# =========================================================================================
# Test routine
# =========================================================================================

# Initialize and load data from the files
db = Database()
db.loadDatabase()
#db.createTestRoomObject()

# Play around with the containing data
print db.getRoom(21).room_issues_[0].issue_id_
print db.getRoom(21).room_name_
print db.getRoom(21).room_information_in_pixel_
print db.getRoom(21).room_information_in_meter_
print db.robot_properties_.exploration_coverage_radius_
print db.getRoom(21).room_map_filename_
print db.getRoom(42).room_map_filename_
print db.global_settings_.shall_auto_complete_
print db.getRoom(21).room_trashcan_count_

# Discard Changes
db.discardTemporalDatabase()

# Save database
db.saveDatabase(temporal=False)

"""