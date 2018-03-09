#!/usr/bin/env python

# For the room, assignment and robot data information
import database_classes
# For time calculations
from datetime import datetime, date, time, timedelta
# For Point32
from geometry_msgs.msg import Point32
# For map receiving, for image receiving
import cv2
from cv_bridge import CvBridge, CvBridgeError
# For support of the JSON format
import json

# Database class
class Database():
	# Assignments
	assignments_ = []
	# Rooms
	rooms_ = []
	# Robot proprties
	robot_properties_ = None
	# Global settings
	global_settings_ = None

# =========================================================================================
# Test functions
# =========================================================================================

	# Create a dictionary which contains one room with 2 issues
	def createTestRoomDict(self):
		rooms_dict = {}
		rooms_dict[42] = {
			"room_name": "D3.06",
			"room_id": 42,
			"last_successful_clean_date": datetime.now().strftime("%Y-%m-%d_%H:%M"),
			"last_cleanup_successful": "False",
			"room_map": None,
			"room_coords": [0, 0, 0],
			"room_issues": {
				1: {
					"issue_type": 12,
					"issue_images": ["~/resources/D3.06_20030201_1.jpg", "~/resources/D3.06_20030201_2.jpg"],
					"issue_coords": [0, 0, 0],
					"issue_date": datetime.now().strftime("%Y-%m-%d_%H:%M"),
					"issue_id": 1
				},
				2: {
					"issue_type": 4,
					"issue_images": ["~/resources/D3.06_20130302_1.jpg", "~/resources/D3.06_20130302_2.jpg"],
					"issue_coords": [0, 0, 0],
					"issue_date": datetime.now().strftime("%Y-%m-%d_%H:%M"),
					"issue_id": 2
				}
			}
		}
		return rooms_dict

	# Add a test room object to the rooms list
	def createTestRoomObject(self):
		# Create room test issue 1
		issue1 = database_classes.RoomIssue()
		issue1.issue_id_ = 1
		issue1.issue_coords_ = None
		issue1.issue_date_ = datetime.now(),
		issue1.issue_images_ = ["~/testordner/OBild1", "~/testordner/OBild2"]
		issue1.issue_type_ = 21
		# Create room test issue 2
		issue2 = database_classes.RoomIssue()
		issue2.issue_id_ = 2
		issue2.issue_coords_ = None
		issue2.issue_date_ = datetime.now(),
		issue2.issue_images_ = ["~/testordner/OBild3", "~/testordner/OBild4"]
		issue2.issue_type_ = 11
		# Create test room
		test_room = database_classes.RoomItem()
		test_room.room_issues_.append(issue1)
		test_room.room_issues_.append(issue2)
		test_room.room_id_ = 42
		test_room.last_cleanup_successful_ = False
		test_room.last_successful_clean_date_ = None
		test_room.room_map_ = None
		test_room.room_center_coords_ = None
		test_room.room_name_ = "Kitchen"
		# Add room to the room list
		self.rooms_.append(test_room)
	
	
	# Create a test assignment dictionary
	def createTestAssignmentDict(self):
		assignment_dict = {}
		assignment_dict[42] = {
			"assignment_name": "Assignment Number Two",
			"scheduled_rooms_cleaning": [5,6,7,8],
			"last_completed_clean": datetime.now().strftime("%Y-%m-%d_%H:%M")
		}
		return assignment_dict


	# Add a test assignment object to the assignments_ list
	def createTestAssignmentObject(self):
		test_assignment = database_classes.AssignmentItem()
		test_assignment.assignment_name_ = "Assignment Number One"
		test_assignment.scheduled_rooms_cleaning_ = [1,2,3,4]
		test_assignment.last_completed_clean_ = datetime.now()
		self.assignments_.append(test_assignment)

# =========================================================================================
# Private methods
# =========================================================================================

	def updateGlobalSettings(self, dict):
		self.global_settings_ = database_classes.GlobalSettings()
		self.global_settings_.shall_auto_complete_ = dict.get("shall_auto_complete")
		self.global_settings_.max_aux_time_ = dict.get("max_aux_time")
		self.global_settings_.assignment_timedelta_ = dict.get("assignment_timedelta")

	def getGlobalSettingsDictFromGlobalSettings(self):
		global_settings_dict = {}
		global_settings_dict["shall_auto_complete"] = self.global_settings_.shall_auto_complete_
		global_settings_dict["max_aux_time"] = self.global_settings_.max_aux_time_
		global_settings_dict["assignment_timedelta"] = self.global_settings_.assignment_timedelta_
		return global_settings_dict

	def updateRobotProperties(self, dict):
		self.robot_properties_ = database_classes.RobotProperties()
		# Exploration server constants
		self.robot_properties_.exploration_robot_radius_ = dict.get("exploration_robot_radius")
		self.robot_properties_.exploration_coverage_radius_ = dict.get("exploration_coverage_radius")
		e_fov_list = dict.get("exploration_field_of_view")
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
	def updateRoomsList(self, dict):
		self.rooms_ = []
		for room_key in dict:
			room_issues = []
			current_room = database_classes.RoomItem()
			# Get all issues of a room and get all properties of those issues
			issues_dict = dict.get(room_key).get("room_issues")
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
				date_str = datetime.strptime(issues_dict.get(issue_key).get("issue_date"), "%Y-%m-%d_%H:%M")
				current_issue.issue_date_ = date_str
				# Append current room issue to the room_issues list
				room_issues.append(current_issue)
			current_room.room_issues_ = room_issues
			# Get the name of the room
			current_room.room_name_ = dict.get(room_key).get("room_name")
			# Get the ID of the room
			current_room.room_id_ = dict.get(room_key).get("room_id")
			# Get the position ID of the room
			current_room.room_position_id_ = dict.get(room_key).get("room_position_id")
			# Get the floor of the room
			current_room.room_floor_id_ = dict.get(room_key).get("room_floor_id")
			# Get the building ID of the room
			current_room.room_building_id_ = dict.get(room_key).get("room_building_id")
			# Get the territory the room is in
			current_room.room_territory_id_ = dict.get(room_key).get("room_territory_id")
			# Get the map of the room
			current_room.room_map_ = dict.get(room_key).get("room_map")
			# Get an open cv representation of the map or None if there is no map
			if (current_room.room_map_ != None):
				room_map_file_path = str(self.extracted_file_path) + str("resources/maps/") + str(current_room.room_map_)
				map_opencv = cv2.imread(room_map_file_path, 0)
				bridge = CvBridge()
				current_room.room_map_data_ = bridge.cv2_to_imgmsg(map_opencv, encoding = "mono8")
			else:
				current_room.room_map_data = None
			# Get the room center coordinates
			room_center_coords_list = dict.get(room_key).get("room_center_coords")
			current_room.room_center_coords_ = Point32(x=room_center_coords_list[0], y=room_center_coords_list[1], z=room_center_coords_list[2])
			# Get the room surface type
			current_room.room_surface_type_ = dict.get(room_key).get("room_surface_type")
			# Get the cleaning method of the room
			current_room.room_cleaning_method_ = dict.get(room_key).get("room_cleaning_method")
			# Get the room surface area
			current_room.room_surface_area_ = dict.get(room_key).get("room_surface_area")
			# Get the room trashcan count
			current_room.room_trashcan_count_ = dict.get(room_key).get("room_trashcan_count")
			# Get the information if the last clean has not yet been completed
			current_room.last_cleanup_successful_ = dict.get(room_key).get("last_cleanup_successful")
			# Get the last successful clean date if there is any, otherwise set None
			date_str = dict.get(room_key).get("last_successful_clean_date")
			if (date_str != None):
				current_room.last_successful_clean_date_ = datetime.strptime(date_str, "%Y-%m-%d_%H:%M")
			else:
				current_room.last_successful_clean_date_ = None
			# Append current room object to the rooms_ list
			self.rooms_.append(current_room)


	# Get a dictionary representation of rooms_
	def getRoomsDictFromRoomsList(self):
		room_dict = {}
		for current_room in self.rooms_:
			# Check if current_room is a room 
			if (isinstance(current_room, database_classes.RoomItem) == True):
				# Make a dict of the issues of current_room
				issues_dict = {}
				for current_issue in current_room.room_issues_:
					# Check if current_issue is an issue
					if (isinstance(current_issue, database_classes.RoomIssue) == True):
						# Fill in a string representation of the date
						date_str = current_issue.issue_date_.strftime("%Y-%m-%d_%H:%M")
						# Fill in the issue coordinates
						ic_x = current_issue.issue_coords_.x
						ic_y = current_issue.issue_coords_.y
						ic_z = current_issue.issue_coords_.z
						issue_coords_list = [ic_x, ic_y, ic_z]
						# Fill in the dictionary with the data
						issues_dict[str(current_issue.issue_id_)] = {
							"issue_id": current_issue.issue_id_,
							"issue_type": current_issue.issue_type_,
							"issue_images": current_issue.issue_images_,
							"issue_coords": issue_coords_list,
							"issue_date": date_str
						}
					else:
						print "[FATAL]: An element in issues array is not an issue object!"
				# Fill in the last successful clean date if there is any, otherwise fill in None
				if (current_room.last_successful_clean_date_ != None):
					date_str = current_room.last_successful_clean_date_.strftime("%Y-%m-%d_%H:%M")
				else:
					date_str = None
				# Fill in the room center coordinates
				if (current_room.room_center_coords_ != None):
					rcc_x = current_room.room_center_coords_.x
					rcc_y = current_room.room_center_coords_.y
					rcc_z = current_room.room_center_coords_.z
					room_center_coords_list = [rcc_x, rcc_y, rcc_z]
				else:
					room_center_coords_list = [-1, -1, -1]
				# Fill the dictionary with the data
				room_dict[str(current_room.room_id_)] = {
					"room_id": current_room.room_id_,
					"room_name": current_room.room_name_,
					"room_position_id": current_room.room_position_id_,
					"room_floor_id": current_room.room_floor_id_,
					"room_building_id": current_room.room_building_id_,
					"room_territory_id": current_room.room_territory_id_,
					"last_successful_clean_date": date_str,
					"last_cleanup_successful": current_room.last_cleanup_successful_,
					"room_issues": issues_dict,
					"room_map": current_room.room_map_,
					"room_center_coords": room_center_coords_list,
					"room_surface_type": current_room.room_surface_type_,
					"room_cleaning_method": current_room.room_cleaning_method_,
					"room_surface_area": current_room.room_surface_area_,
					"room_trashcan_count": current_room.room_trashcan_count_
				}
			else:
				print "[FATAL]: An element in rooms_ array is not a room object!"
		return room_dict


	# Make assignments_ contain all the rooms stated in the dict parameter
	def updateAssignmentsList(self, dict):
		self.assignments_ = []
		for assignment_key in dict:
			current_assignment = database_classes.AssignmentItem()
			# Get the name of the previous assignment
			current_assignment.prev_assignment_ = dict.get(assignment_key).get("prev_assignment")
			# Get the name of the assignment
			current_assignment.assignment_name_ = dict.get(assignment_key).get("assignment_name")
			# Get the week type the assignment is active
			current_assignment.assignment_week_type_ = dict.get(assignment_key).get("assignment_week_type")
			# Get week day the assignment is active
			current_assignment.assignment_week_day_ = dict.get(assignment_key).get("assignment_week_day")
			# Get the list of room IDs for cleaning which are scheduled in this assignment
			current_assignment.scheduled_rooms_cleaning_ = dict.get(assignment_key).get("scheduled_rooms_cleaning")
			# Get the list of room IDs for trashcan which are scheduled in this assignment
			current_assignment.scheduled_rooms_trashcan_ = dict.get(assignment_key).get("scheduled_rooms_trashcan")
			# Get the RoomItem representation behind the ID for all rooms to clean
			for room_id in current_assignment.scheduled_rooms_cleaning_:
				current_assignment.scheduled_rooms_cleaning_data_.append(self.getRoom(room_id))
			# Get the RoomItem representation behind the ID for all rooms to empty the trashcan
			for room_id in current_assignment.scheduled_rooms_trashcan_:
				current_assignment.scheduled_rooms_trashcan_data_.append(self.getRoom(room_id))
			# Get a date string or None if there is no date
			date_str = dict.get(assignment_key).get("last_completed_clean")
			if (date_str != None):
				current_assignment.last_completed_clean_ = datetime.strptime(date_str, "%Y-%m-%d_%H:%M")
			else:
				current_assignment.last_completed_clean_ = None
			# Append current_assignment to assignments_ list
			self.assignments_.append(current_assignment)


	# Get a dictionary representation of assignments_
	def getAssignmentsDictFromAssignmentsList(self):
		assignment_dict = {}
		for current_assignment in self.assignments_:
			# Check if current_assignment is an AssignmentItem object
			if (isinstance(current_assignment, database_classes.AssignmentItem)):
				# Get the last completed clean date if there is any, otherwise set it as None
				if (current_assignment.last_completed_clean_ != None):
					date_str = current_assignment.last_completed_clean_.strftime("%Y-%m-%d_%H:%M")
				else:
					date_str = None
				# Fill the dictionary with the data
				assignment_dict[current_assignment.assignment_name_] = {
					"prev_assignment": current_assignment.prev_assignment_,
					"assignment_name": current_assignment.assignment_name_,
					"assignment_week_type": current_assignment.assignment_week_type_,
					"assignment_week_day": current_assignment.assignment_week_day_,
					"scheduled_rooms_cleaning": current_assignment.scheduled_rooms_cleaning_,
					"scheduled_rooms_trashcan": current_assignment.scheduled_rooms_trashcan_,
					"last_completed_clean": date_str
				}
			else:
				print "[FATAL]: An element in assignments_ array is not an assignment object!"
		return assignment_dict



# =========================================================================================
# Public methods
# =========================================================================================

	# Constructor method
	def __init__(self, extracted_file_path=""):
		if (extracted_file_path != None):
			self.extracted_file_path = extracted_file_path

	# Load database data from files
	def loadDatabase(self):
		# Load the room data
		file = open(self.extracted_file_path + str("resources/json/rooms.json"), "r").read()
		rooms_dict = json.loads(file)
		self.updateRoomsList(rooms_dict)
		# Load the assignment data
		file = open(self.extracted_file_path + str("resources/json/assignments.json"), "r").read()
		assignments_dict = json.loads(file)
		self.updateAssignmentsList(assignments_dict)
		# Load the robot properties
		file = open(self.extracted_file_path + str("resources/json/robot_properties.json"), "r").read()
		robot_properties_dict = json.loads(file)
		self.updateRobotProperties(robot_properties_dict)
		# Load the global settings
		file = open(self.extracted_file_path + str("resources/json/global_settings.json"), "r").read()
		global_settings_dict = json.loads(file)
		self.updateGlobalSettings(global_settings_dict)

	# Save database data to files
	def saveDatabase(self):
		# Save the room data
		rooms_dict = self.getRoomsDictFromRoomsList()
		rooms_text = json.dumps(rooms_dict, indent=4, sort_keys=True)
		file = open(self.extracted_file_path + str("resources/json/rooms.json"), "w")
		file.write(rooms_text)
		# Save the assignment data
		assignments_dict = self.getAssignmentsDictFromAssignmentsList()
		assignments_text = json.dumps(assignments_dict, indent=4, sort_keys=True)
		file = open(self.extracted_file_path + str("resources/json/assignments.json"), "w")
		file.write(assignments_text)
		# Save global settings
		global_settings_dict = self.getGlobalSettingsDictFromGlobalSettings()
		global_settings_text = json.dumps(global_settings_dict, indent=4, sort_keys=True)
		file = open(self.extracted_file_path + str("resources/json/global_settings.json"), "w")
		file.write(global_settings_text)
		# Save robot properties? No.....
		# [...]


	# Retreive a room by providing a room_id
	def getRoom(self, room_id):
		result = None
		for i in range(len(self.rooms_)):
			if (self.rooms_[i].room_id_ == room_id):
				result = self.rooms_[i]
		return result

	# Retreive a room by providing a position id and a floor id
	def getRoomByPosFloor(self, pos_id, floor_id):
		result = None
		for room in self.rooms_:
			if ((room.room_position_id_ == pos_id) and (room.room_floor_id_ == floor_id)):
				result = room
				break
		return result

	# Retreive an assignment by providing a week day and week type
	def getAssignmentByWeekTypeDay(self, week_type, week_day):
		result = None
		for assignment in self.assignments_:
			if ((assignment.assignment_week_type_ == week_type) and (assignment.assignment_week_day_ == week_day)):
				result = assignment
				break
		return result


"""


# =========================================================================================
# Test routine
# =========================================================================================

# Initialize and load data from the files
db = Database()
db.loadDatabase()
#db.createTestRoomObject()
#db.createTestAssignmentObject()

# Play around with the containing data
print db.getRoom(21).room_issues_[0].issue_id_
print db.getRoom(21).room_name_
print db.robot_properties_.exploration_coverage_radius_
print db.getRoom(21).room_map_
print db.getRoom(42).room_map_
print db.getAssignment(42).scheduled_rooms_cleaning_data_[1].room_id_
print db.getAssignment(21).assignment_name_
print db.global_settings_.shall_auto_complete_
print db.getRoom(21).room_trashcan_count_

# Save database
db.saveDatabase()

"""