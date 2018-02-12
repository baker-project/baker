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
			"last_cleanup_unsuccessful": "False",
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
		test_room.last_cleanup_unsuccessful_ = False
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
			"assignment_id": 21,
			"scheduled_rooms": [5,6,7,8],
			"clean_dates": [datetime.now().strftime("%Y-%m-%d_%H:%M"), datetime.now().strftime("%Y-%m-%d_%H:%M")],
			"last_completed_clean": datetime.now().strftime("%Y-%m-%d_%H:%M"),
			"clean_interval": 0,
			"shall_auto_complete": True
		}
		return assignment_dict


	# Add a test assignment object to the assignments_ list
	def createTestAssignmentObject(self):
		test_assignment = database_classes.AssignmentItem()
		test_assignment.assignment_name_ = "Assignment Number One"
		test_assignment.assignment_id_ = 42
		test_assignment.scheduled_rooms_ = [1,2,3,4]
		test_assignment.clean_dates_ = []
		test_assignment.last_completed_clean_ = datetime.now()
		test_assignment.clean_interval_ = 0
		test_assignment.shall_auto_complete_ = True
		self.assignments_.append(test_assignment)

# =========================================================================================
# Private methods
# =========================================================================================

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
				#current_issue.issue_coords_ = None
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
			# Get the map of the room
			current_room.room_map_ = dict.get(room_key).get("room_map")
			# Get the room center coordinates
			room_center_coords_list = dict.get(room_key).get("room_center_coords")
			current_room.room_center_coords_ = Point32(x=room_center_coords_list[0], y=room_center_coords_list[1], z=room_center_coords_list[2])
			# Get the information if the last clean has not yet been completed
			current_room.last_cleanup_unsuccessful_ = dict.get(room_key).get("last_cleanup_unsuccessful")
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
				rcc_x = current_room.room_center_coords_.x
				rcc_y = current_room.room_center_coords_.y
				rcc_z = current_room.room_center_coords_.z
				room_center_coords_list = [rcc_x, rcc_y, rcc_z]
				# Fill the dictionary with the data
				room_dict[str(current_room.room_id_)] = {
					"room_id": current_room.room_id_,
					"room_name": current_room.room_name_,
					"last_successful_clean_date": date_str,
					"last_cleanup_unsuccessful": current_room.last_cleanup_unsuccessful_,
					"room_issues": issues_dict,
					"room_map": current_room.room_map_,
					"room_center_coords": room_center_coords_list
				}
			else:
				print "[FATAL]: An element in rooms_ array is not a room object!"
		return room_dict


	# Make assignments_ contain all the rooms stated in the dict parameter
	def updateAssignmentsList(self, dict):
		self.assignments_ = []
		for assignment_key in dict:
			current_assignment = database_classes.AssignmentItem()
			# Get the name of the assignment
			current_assignment.assignment_name_ = dict.get(assignment_key).get("assignment_name")
			# Get the assignment ID
			current_assignment.assignment_id_ = dict.get(assignment_key).get("assignment_id")
			# Get the list of room IDs which are scheduled in this assignment
			current_assignment.scheduled_rooms_ = dict.get(assignment_key).get("scheduled_rooms")
			# Get a date string or None if there is no date
			date_str = dict.get(assignment_key).get("last_completed_clean")
			if (date_str != None):
				current_assignment.last_completed_clean_ = datetime.strptime(date_str, "%Y-%m-%d_%H:%M")
			else:
				current_assignment.last_completed_clean_ = None
			# Get the clean interval
			clean_interval_int = dict.get(assignment_key).get("clean_interval")
			current_assignment.clean_interval_ = timedelta(days = clean_interval_int)
			# Get the explicit clean dates
			current_assignment.clean_dates_ = []
			dates_str = dict.get(assignment_key).get("clean_dates")
			for date in dates_str:
				current_assignment.clean_dates_.append(datetime.strptime(date, "%Y-%m-%d_%H:%M"))
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
				# Make a string list of all explicit cleaning dates
				expl_dates = []
				for clean_date in current_assignment.clean_dates_:
					expl_dates.append(clean_date.strftime("%Y-%m-%d_%H:%M"))
				# Get the clean interval
				clean_interval_int = current_assignment.clean_interval_.days
				# Fill the dictionary with the data
				assignment_dict[str(current_assignment.assignment_id_)] = {
					"assignment_name": current_assignment.assignment_name_,
					"assignment_id": current_assignment.assignment_id_,
					"scheduled_rooms": current_assignment.scheduled_rooms_,
					"last_completed_clean": date_str,
					"clean_interval": clean_interval_int,
					"clean_dates": expl_dates,
					"shall_auto_complete": current_assignment.shall_auto_complete_
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
		file = open(self.extracted_file_path + str("rooms.json"), "r").read()
		rooms_dict = json.loads(file)
		self.updateRoomsList(rooms_dict)
		# Load the assignment data
		file = open(self.extracted_file_path + str("assignments.json"), "r").read()
		assignments_dict = json.loads(file)
		self.updateAssignmentsList(assignments_dict)
		# Load the robot properties
		file = open(self.extracted_file_path + str("robot_properties.json"), "r").read()
		robot_properties_dict = json.loads(file)
		self.updateRobotProperties(robot_properties_dict)

	# Save database data to files
	def saveDatabase(self):
		# Save the room data
		rooms_dict = self.getRoomsDictFromRoomsList()
		rooms_text = json.dumps(rooms_dict, indent=4, sort_keys=True)
		file = open(self.extracted_file_path + str("rooms.json"), "w")
		file.write(rooms_text)
		# Save the assignment data
		assignments_dict = self.getAssignmentsDictFromAssignmentsList()
		assignments_text = json.dumps(assignments_dict, indent=4, sort_keys=True)
		file = open(self.extracted_file_path + str("assignments.json"), "w")
		file.write(assignments_text)
		# Save robot properties? No.....
		# [...]


	# Retreive a room by providing a room_id
	def getRoom(self, room_id):
		result = None
		for i in range(len(self.rooms_)):
			if (self.rooms_[i].room_id_ == room_id):
				result = self.rooms_[i]
		return result

	# Retreive an assignment by providing an assignment_id
	def getAssignment(self, assignment_id):
		result = None
		for i in range(len(self.assignments_)):
			if(self.assignments_[i].assignment_id_ == assignment_id):
				result = self.assignments_[i]
		return result

	# Retrieve a cv_bridge room map by providing a room_id
	def loadRoomMapAsCVBridge(self, room_id):
		wanted_room = None
		for room in self.rooms_:
			if (room.room_id_ == room_id):
				wanted_room = room
		if (wanted_room != None):
			path = str(self.extracted_file_path) + str("resources/maps/") + str(wanted_room.room_map_)
			map_opencv = cv2.imread(path)
			bridge = CvBridge()
			return bridge.self.bridge_.cv2_to_imgmsg(map_opencv, encoding = "mono8")
		else:
			return None





# =========================================================================================
# Test routine
# =========================================================================================

# Initialize and load data from the files
db = Database()
db.loadDatabase()

# Play around with the containing data
print db.getRoom(21).room_issues_[0].issue_id_
print db.getRoom(21).room_name_
print db.robot_properties_.exploration_coverage_radius_
print db.getRoom(21).room_map_
print db.getRoom(42).room_map_
#db.createTestRoomObject()
#db.createTestAssignmentObject()
print db.getAssignment(21).assignment_name_

# Save database
db.saveDatabase()