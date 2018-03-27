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
	# Contains all due rooms
	due_rooms_ = []
	# Contains all overdue rooms
	overdue_rooms_ = []

	# ===============================================================================
	# STATIC METHODS
	# ===============================================================================

	@staticmethod
	def getTodaysWeekType():
		weekNumber = date.today().isocalendar()[1]
		return weekNumber % 2

	@staticmethod
	def getTodaysWeekDay():
		return date.today().weekday()

	@staticmethod
	def getTodaysScheduleIndex():
		return DatabaseHandler.getTodaysWeekType() * 7 + DatabaseHandler.getTodaysWeekDay()

	@staticmethod
	def noCleaningYetPerformed(room):
		trashcan_date = room.room_cleaning_timestamps_[0]
		dry_date = room.room_cleaning_timestamps_[1]
		wet_date = room.room_cleaning_timestamps_[2]
		method = room.room_cleaning_method_
		result = False
		if (method == 0) and (dry_date == None):
			result = True
		elif (method == 1) and (wet_date == None):
			result = True
		elif (method == 2) and ((wet_date == None) or (dry_date == None)):
			result = True
		return result

	
	# Get the room information in pixel
	def getMapAndRoomInformationInPixel(self, rooms_array):
		room_information_in_pixel = []
		bridge = CvBridge()
		segmentation_id = 0
		# Get the dimension of the image an create the temp map
		complete_map_opencv = bridge.imgmsg_to_cv2(self.database_.global_map_data_.map_image_, desired_encoding = "passthrough")
		image_height, image_width = complete_map_opencv.shape
		tmp_map_opencv = np.zeros((image_height, image_width), np.uint8)
		for room in rooms_array:
			# Get an OPENCV representation of the image
			room_map_opencv = bridge.imgmsg_to_cv2(room.room_map_data_, desired_encoding = "passthrough")
			# Add the room to the final map
			for x in range(image_width):
				for y in range(image_height):
					if (room_map_opencv[y, x] == 255):
						tmp_map_opencv[y, x] = segmentation_id + 1
			# Get room_information_in_pixels
			room_information_in_pixel.append(room.room_information_in_pixel_)
			segmentation_id = segmentation_id + 1
		cv2.imshow('image', tmp_map_opencv)
		segmented_map = bridge.cv2_to_imgmsg(tmp_map_opencv, encoding = "mono8")
		return room_information_in_pixel, segmented_map


	# ===============================================================================
	# OBJECT SPECIFIC METHODS
	# ===============================================================================


	# Get the room information in meter
	def getRoomInformationInMeter(self, rooms_array):
		room_information_in_meter = []
		for room in rooms_array:
			room_information_in_meter.append(room.room_information_in_meter_)
		return room_information_in_meter

	def __init__(self, database):
		self.database_ = database

	# Check via date if an assignment is due
	def assignmentDateIsDue(self, datetime_stamp):
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
	def getAllDueRooms(self):
		# A room is due if one of its timestamps indicates this
		self.due_rooms_ = []
		today_index = self.getTodaysScheduleIndex()
		for room in self.database_.rooms_:
			if (room.room_scheduled_days[today_index] != ""):
				self.due_rooms_.append(room)


	# Method for extracting all overdue rooms from the due assignment
	def getAllOverdueRooms(self):
		# A room is overdue if
		# 1. The room is cleaned wet & the timestamp for wet cleaning is overdue
		# 2. The room is cleaned dry & the timestamp for dry cleaning is overdue
		# 3. The room is cleaned in both ways & one of the timestamps stated above is overdue
		# 4. The rooms trashcan timestamp is overdue

		# A room is not overdue if
		# 1. The last trashcan emptying has been done
		# 2. The last cleaning process has been done
		
		# Therefore, remove all rooms which are fine.

		# Get all rooms which are not listed in the due rooms list yet
		potential_overdue_rooms = []	
		for room in self.database_.rooms_:
			if not(room in self.due_rooms_):
				self.overdue_rooms_.append(room)
		today_index = 0
		current_schedule_index = today_index - 1
		day_delta = 1
		while (current_schedule_index != today_index):
			# Handle the case that today_index is a monday of an even week
			if (current_schedule_index == -1):
				current_schedule_index = 13
				continue
			# Iterate through all potential rooms
			for room in self.overdue_rooms_:
				if (room.room_scheduled_days_[current_schedule_index != ""]):
					trashcan_date = room.room_cleaning_timestamps_[0]
					dry_date = room.room_cleaning_timestamps_[1]
					wet_date = room.room_cleaning_timestamps_[2]
					
			current_schedule_index = current_schedule_index - 1
			day_delta = day_delta + 1
			

	# Method for sorting a list of rooms after the cleaning method
	def sortRoomsList(self, rooms_list):
		rooms_wet_cleaning = []
		rooms_dry_cleaning = []
		for room in rooms_list:
			# dry cleaning
			if (room.room_cleaning_method_ == 0):
				rooms_dry_cleaning.append(room)
			# wet cleaning
			if (room.room_cleaning_method_ == 1):
				rooms_wet_cleaning.append(room)
			# both
			if (room.room_cleaning_method_ == 2):
				rooms_dry_cleaning.append(room)
				rooms_wet_cleaning.append(room)
		return rooms_dry_cleaning, rooms_wet_cleaning


	# Method for setting a room as completed
	def checkoutCompletedRoom(self, room, assignment_type):

		# Save all changes to the database
		self.database_.saveDatabase()

	# Method to run if a change in the database shall be applied
	# Applied changes can be discarded
	def applyChangesToDatabase(self):
		self.database_.saveDatabase()

	# Method to run after all cleaning operations were performed
	def cleanFinished(self):
		self.database_.saveDatabase(temporal=False)



dbh = DatabaseHandler(None)
dbh.getAllOverdueRooms()