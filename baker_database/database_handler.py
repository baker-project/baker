#!/usr/bin/env python

# For database
import database
import database_classes
# For date and time calculations
import datetime
from datetime import date
# For room information
from ipa_building_msgs.msg import *
import geometry_msgs
# For map generation
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


class DatabaseHandler():
	database_ = None
	# The assignment which is due today
	due_assignment_ = None
	# Overdue assignments
	overdue_assignments_ = []
	# Contains all due cleaning rooms from the due assignments and removes them right after they were cleaned
	due_rooms_cleaning_ = []
	# Contains all overdue cleaning rooms and removed them right after they were cleaned
	overdue_rooms_cleaning_ = []
	# Contains all due trashcan rooms from the due assignments and removes them right after they were cleaned
	due_rooms_trashcan_ = []
	# Contains all overdue trashcan rooms and removed them right after they were cleaned
	overdue_rooms_trashcan_ = []

	# ===============================================================================
	# STATIC METHODS
	# ===============================================================================

	def getTodaysWeekType():
		weekNumber = date.today().isocalendar()[1]
		return weekNumber % 2

	def getTodaysWeekDay():
		return date.today().weekday()

	# Get the room information in pixel
	def getMapAndRoomInformationInPixel(rooms_array):
		room_information_in_pixel = []
		bridge = CvBridge()
		segmentation_id = 0
		for room in rooms_array:
			# Get an OPENCV representation of the image
			room_map_opencv = bridge.imgmsg_to_cv2(room.room_map_data_, desired_encoding = "passthrough")
			# Get the dimension of the map through the first map image
			if (segmentation_id == 0):
				image_height, image_width = room_map_opencv.shape
				tmp_map_opencv = np.zeros((image_height, image_width), np.uint8)
			# Add the room to the final map
			for x in range(image_width):
				for y in range(image_height):
					if (room_map_opencv[y, x] == 255):
						tmp_map_opencv[y, x] = segmentation_id + 1
			# Get room_information_in_pixels
			room_information_in_pixel.append(room.room_information_in_pixel_)
			segmentation_id = segmentation_id + 1
		segmented_map = bridge.cv2_to_imgmsg(tmp_map_opencv, encoding = "mono8")
		return room_information_in_pixel, segmented_map


	# ===============================================================================
	# OBJECT SPECIFIC METHODS
	# ===============================================================================


	# Get the room information in meter
	def getRoomInformationInMeter(rooms_array):
		room_information_in_meter = []
		for room in rooms_array:
			room_information_in_meter.append(room.room_information_in_meter_)
		return room_information_in_meter

	def __init__(self, database):
		self.database_ = database

	# Check via date if an assignment is due
	def assignmentDateIsDue(datetime_stamp):
		a_timedelta = datetime.timedelta(days=self.database_.global_settings_.assignment_timedelta_)
		return ((datetime_stamp != None) and (datetime_stamp - date.today > a_timedelta))

	# Check via date if a room must be cleaned
	# A past assigned room is overdue if 
	# 1. Its date is not None
	# 2. The room would have been due in the past according to the given last successful clean date
	def roomDateIsOverdue(self, datetime_stamp, shift_days):
		r_shift_days = datetime.timedelta(days=shift_days)
		assignment_timedelta = self.database_.global_settings_.assignment_timedelta_
		return ((datetime_stamp != None) and (datetime_stamp - date.today - r_shift_days > assignment_timedelta))

	# Method for extracting all due rooms from the due assignment
	def getAllDueAssignmentsAndRooms(self):

		# Get the due assignment
		for assignment in self.database_.assignments_:
			if ((assignment.assignment_week_type_ == getTodaysWeekType()) 
			and (assignment.assignment_week_day_ == getTodaysWeekDay())):
				self.due_assignment_ = assignment
				break

		# Get all rooms unfinished from the due assignment and put them in the respective room array
		for room in self.due_assignment_.scheduled_rooms_cleaning_data_:
			self.due_rooms_cleaning_.append(room)
		for room in self.due_assignment_.scheduled_rooms_trashcan_data_:
			self.due_rooms_trashcan_.append(room)

		# If wanted: Get all overdue assignments in correct order
		# But: Assignments with date "None" shall not be added! They are new and therefore they could not have been done!
		if (self.database_.global_settings_.do_auto_complete_ == True):
			it_assignment = assignment.prev_assignment_
			while (it_assignment != assignment):
			# Find those assignments whose time stamp is further than 14 days in the past
				if (self.assignmentDateIsDue(it_assignment.last_successful_clean_date_) == True):
					self.overdue_assignments_.append(assignment)
				it_assignment = it_assignment.prev_assignment_
			# Get the rooms which should be contained in the overdue array. This is the case if...
			# 1. They are not already in due or overdue array 
			# 2. In case of trashcan: If the room is not yet in one of the arrays for cleaning
			# 3. If the date requires it
			for i in range(len(self.overdue_assignments_)):
				it_assignment = self.overdue_assignments_[i]
				for room in assignment.scheduled_rooms_cleaning_data_:
					if (not(room in self.overdue_rooms_cleaning_) 
					and (self.roomDateIsOverdue(room.last_successful_clean_date_, i)) 
					and not(room in self.due_rooms_cleaning_)):
						self.overdue_rooms_cleaning_.append(room)
				for room in assignment.scheduled_rooms_trashcan_data_:
					if (not(room in self.overdue_rooms_trashcan_) 
					and not (room in self.due_rooms_cleaning_))
					and (self.roomDateIsOverdue(room.last_successful_clean_date_, i)) 
					and not(room in self.due_rooms_trashcan_):
						self.overdue_rooms_trashcan_.append(room)


	# Method for setting a room as completed
	def checkoutCompletedRoom(self, room, is_overdue, is_trashcan):
		# Set values in the database
		if (is_trashcan == True):
			room.last_successful_trashcan_date_ = datetime.datetime.now()
		else:
			room.last_successful_cleaning_date_ = datetime.datetime.now()
		# Remove room from the respective list
		if (is_overdue == True):
			if (is_trashcan == True):
				self.overdue_rooms_trashcan_.remove(room)
			else:
				self.overdue_rooms_cleaning_.remove(room)
		else:
			if (is_trashcan == True):
				self.due_rooms_trashcan_.remove(room)
			else:
				self.due_rooms_cleaning_.remove(room)
		# Save all changes to the database
		self.database_.saveDatabase()

	# Method to run if a change in the database shall be applied
	# Applied changes can be discarded
	def applyChangesToDatabase(self):
		self.database_.saveDatabase()

	# Method to run after all cleaning operations were performed
	def cleanFinished(self):
		self.database_.saveDatabase(temporal=False)
