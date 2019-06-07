#!/usr/bin/env python

import database
import database_classes
import rospy
# For json
import json
# For Map Receiving
from std_msgs.msg import String
import baker_msgs.srv
# For Segmentation
import actionlib
from ipa_building_msgs.msg import *
# For images
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
# For Point32
from geometry_msgs.msg import Point32
# For timedelta
from datetime import timedelta
# For CSV creation
import csv

class DatabaseCreator():

	def openCv2CvBridge(self, opencv_image):
		return self.bridge_.cv2_to_imgmsg(opencv_image, encoding = "mono8")

	def cvBridge2OpenCv(self, cvbridge_image):
		return self.bridge_.imgmsg_to_cv2(cvbridge_image, desired_encoding = "passthrough")


	def __init__(self):
		self.bridge_ = CvBridge()
		self.map_receiving_service_str_ = '/map_management_client/get_map_image'
		self.map_segmented_receiving_service_str_ = '/map_management_client/get_map_segmented_image'
		self.map_segmentation_service_str_ = '/room_segmentation/room_segmentation_server'
		self.map_segmentation_algorithm_ = 99
		self.robot_radius_ = 0.2875

	def createDatabase(self):
		self.database_ = database.Database()

	def createSegmentedMap(self):

		# Receive map
		# ===========

		# receive the navigation map in sensor_msgs/Image format
		rospy.wait_for_service(self.map_receiving_service_str_)
		try:
			get_map = rospy.ServiceProxy(self.map_receiving_service_str_, baker_msgs.srv.GetMap)
			self.map_data_ = get_map()
		except rospy.ServiceException, e:
			print "Map-Receiving-Service call failed: %s" % e
		print "Map received with resolution: ", self.map_data_.map_resolution, " and origin: ", self.map_data_.map_origin
		
		"""
		# optionally receive the segmented map in sensor_msgs/Image format
		if (self.map_segmented_receiving_service_str_ != None):
			self.printMsg("Waiting for service " + str(self.map_segmented_receiving_service_str_) + " to become available ...")
			try:
				rospy.wait_for_service(self.map_segmented_receiving_service_str_, timeout=3.0)
				get_map_segmented = rospy.ServiceProxy(self.map_segmented_receiving_service_str_, baker_msgs.srv.GetMap)
				self.map_segmented_data_ = get_map_segmented()
				print "Segmented map received with resolution: ", self.map_segmented_data_.map_resolution, " and origin: ", self.map_segmented_data_.map_origin
			except rospy.ServiceException, e:
				print "No segmented map available: %s" % e
				self.map_segmented_data_ = None
		"""


		# Segment map
		# ===========

		segmentation_goal = MapSegmentationGoal()
		segmentation_goal.input_map = self.map_data_.map
		segmentation_goal.map_resolution = self.map_data_.map_resolution
		segmentation_goal.map_origin = self.map_data_.map_origin
		segmentation_goal.return_format_in_meter = True
		segmentation_goal.return_format_in_pixel = True
		segmentation_goal.robot_radius = self.robot_radius_
		segmentation_goal.room_segmentation_algorithm = self.map_segmentation_algorithm_
		segmentation_client = actionlib.SimpleActionClient(str(self.map_segmentation_service_str_), MapSegmentationAction)
		segmentation_client.wait_for_server()
		segmentation_client.send_goal(segmentation_goal)
		segmentation_client.wait_for_result()
		self.segmentation_result_ = segmentation_client.get_result()


	def createRoomEntries(self):
		for i in range(len(self.segmentation_result_.room_information_in_pixel)):
			print "Creating room " + str(i)
			cv_image = self.getMapSegmentAsImage(self.cvBridge2OpenCv(self.segmentation_result_.segmented_map), i)
			img_file_name = "map_" + str(i) + ".png"
			cv2.imwrite("resources/maps/" + str(img_file_name), cv_image)
			room = database_classes.RoomItem()
			room.room_name_ = "room_" + str(i)
			room.room_id_ = i
			room.room_position_id_ = "1_" + str(i)
			room.room_floor_id_ = "1st Floor"
			room.room_building_id_ = "Building C"
			room.room_territory_id_ = "42"
			room.room_surface_type_ = 0
			room.room_cleaning_method_ = 0
			room.room_surface_area_ = 3.141
			room.room_trashcan_count_ = 4
			room.room_cleaning_datestamps_ = [None, None, None]
			room.room_issues_ = []
			room.room_map_ = img_file_name
			room.room_map_data_ = self.openCv2CvBridge(cv_image)
			room.room_map_filename_ = img_file_name
			room.room_information_in_pixel_ = self.segmentation_result_.room_information_in_pixel[i]
			room.room_information_in_meter_ = self.segmentation_result_.room_information_in_meter[i]
			room.room_scheduled_days_ = ["x","x","x","x","x","x","x","x","x","x","x","x","x","x"]
			room.open_cleaning_tasks_ = []
			self.database_.rooms_.append(room)




	def saveDatabase(self):
		self.database_.saveRoomDatabase(temporal=False)

		# Save global robot properties
		# ============================

		global_robot_properties_dict = {
			"exploration_coverage_radius": 0.25,
			"exploration_field_of_view": [[0.04035, 0.136], [0.04035, -0.364], [0.54035, -0.364], [0.54035, 0.136]],
			"exploration_header_frame_id": "base_link",
			"exploration_robot_radius": 0.325,
			"path_follow_goal_angle_tolerance": 0.2,
			"path_follow_goal_position_tolerance": 0.5,
			"path_follow_path_tolerance": 0.2,
			"wall_follow_goal_angle_tolerance": 3.14,
			"wall_follow_goal_position_tolerance": 0.4,
			"wall_follow_path_tolerance": 0.2
		}
		global_robot_properties_text = json.dumps(global_robot_properties_dict, indent=4, sort_keys=True)
		file = open("resources/json/robot_properties.json", "w")
		file.write(global_robot_properties_text)

		# Save global settings
		# ====================

		global_settings_dict = {
			"shall_auto_complete": True,
			"max_aux_time": 24,
			"assignment_timedelta": 14
		}
		global_settings_text = json.dumps(global_settings_dict, indent=4, sort_keys=True)
		file = open("resources/json/robot_settings.json", "w")
		file.write(global_settings_text)

		# Save global map data
		# ====================

		map_origin_array = [
			self.map_data_.map_origin.position.x,
			self.map_data_.map_origin.position.y,
			self.map_data_.map_origin.position.z,
			self.map_data_.map_origin.orientation.w,
			self.map_data_.map_origin.orientation.x,
			self.map_data_.map_origin.orientation.y,
			self.map_data_.map_origin.orientation.z
		]
		global_map_data_dict = {
			"map_resolution": self.map_data_.map_resolution,
			"map_origin": map_origin_array,
			"map_header_frame_id": self.map_data_.map.header.frame_id
		}
		global_map_data_text = json.dumps(global_map_data_dict, indent=4, sort_keys=True)
		file = open("resources/json/global_map_data.json", "w")
		file.write(global_map_data_text)
		segmented_map_image_opencv = self.cvBridge2OpenCv(self.segmentation_result_.segmented_map)
		cv2.imwrite("resources/maps/global_map_segmented.png", segmented_map_image_opencv)
		map_image_opencv = self.cvBridge2OpenCv(self.map_data_.map)
		cv2.imwrite("resources/maps/global_map.png", map_image_opencv())

		# Save global application data
		# ============================

		application_data_dict = {
			"last_database_save_successful": True,
			"last_execution_date": None,
			"last_planning_date": [
				None,
				None
			],
			"run_count": 0
		}
		file = open("resources/json/application_data.json", "w")
		file.write(application_data_dict)



	def runDatabaseCreation(self):
		self.createDatabase()
		self.createSegmentedMap()
		self.createRoomEntries()
		self.saveDatabase()


	# TODO: Remove (Not intended)
	def createRoomBook(self):
		file = open(str("csv/ROOMPLAN.csv"), "w")
		file.write("")
		#file.write("Pos.,Etg.,Gebaude,Raum Nr.,Raum-bezeichnung,Reinigungs-bereich,Belag,Reinigungs-methode,Flache,Papierkorbe,m,Mo-Fr p.Einheit,Sa p.Einheit,So p.Einheit,FT p.Tag,Einheit,")
		file = open(str("csv/ROOMPLAN.csv"), "a")
		#file.write("\n")
		for room in self.database_.rooms_:
			file.write(str(room.room_position_id_) + ";")
			file.write(str(room.room_floor_id_) + ";")
			file.write(str(room.room_building_id_) + ";")
			file.write(str(room.room_id_) + ";")
			file.write(str(room.room_name_) + ";")
			file.write("Reinigungsbereich;")
			file.write(str(room.room_surface_type_) + ";")
			file.write(str(room.room_cleaning_method_) + ";")
			file.write(str(room.room_surface_area_) + ";")
			file.write(str(room.room_trashcan_count_) + ";")
			file.write(";")
			file.write("?;")
			file.write("?;")
			file.write("?;")
			file.write("?;")
			file.write("Woche;")
			file.write("\n")


	def createTerritoryPlan(self):
		file = open(str("csv/TERRITORYPLAN.csv"), "w")
		file.write("")
		#file.write(",,,,,,,,,,,,,,,,,,,,,,,")
		file = open(str("csv/TERRITORYPLAN.csv"), "a")
		#file.write("\n")
		#file.write("Rev.,Pos.,Etg.,Raum Nr.,Bez.1,Bez.2,R.-Gr.,Bez.3,Belag,Flache,Reinigungsintervall,Mo,Di,Mi,Do,Fr,Sa,So,Mo,Di,Mi,Do,Fr,Sa,So")
		#file.write("\n")
		for room in self.database_.rooms_:
			file.write(str(room.room_territory_id_) + ";")
			file.write(str(room.room_position_id_) + ";")
			file.write(str(room.room_floor_id_) + ";")
			file.write(str(room.room_id_) + ";")
			file.write("Bezeichnung 1;")
			file.write("Bezeichnung 2;")
			file.write("Raumgruppe;")
			file.write("Bezeichnung 3;")
			file.write(str(room.room_surface_type_) + ";")
			file.write(str(room.room_surface_area_) + ";")
			file.write("INTERVAL_STRING;")
			file.write(";;;;;;;;;;;;;;")
			file.write("\n")



	# Method for returning the segment of the map corresponding to the order number as cv_bridge
	def getMapSegmentAsImage(self, opencv_segmented_map, current_room_index):
		image_height, image_width = opencv_segmented_map.shape
		tmp_map_opencv = np.zeros((image_height, image_width), np.uint8)
		for x in range(image_width):
			for y in range(image_height):
				if (opencv_segmented_map[y, x] == current_room_index + 1):
					tmp_map_opencv[y, x] = 255
		return tmp_map_opencv



# =======================================================================================================
# Calling the database creation

rospy.init_node('database_creation')
database_creator = DatabaseCreator()
database_creator.runDatabaseCreation()
database_creator.createRoomBook()
database_creator.createTerritoryPlan()




