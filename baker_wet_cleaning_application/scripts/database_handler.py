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


	# ===============================================================================
	# OBJECT SPECIFIC METHODS
	# ===============================================================================



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



	# Get the room information in meter
	def getRoomInformationInMeter(self, rooms_array):
		room_information_in_meter = []
		for room in rooms_array:
			room_information_in_meter.append(room.room_information_in_meter_)
		return room_information_in_meter

	def __init__(self, database):
		self.database_ = database

	
	# Method for extracting all due rooms from the due assignment
	# CASE: First run of application, no rooms collected yet today.
	def getAllDueRooms(self):
		# If the application ran already today and the due rooms list is umenpty, this should not run
		if (self.database_.application_data_.last_execution_date_ != None):
			delta = date.today() - self.database_.application_data_.last_execution_date_
			if ((delta.days == 0) and (len(self.due_rooms_) != 0)):
				return
		today_index = self.getTodaysScheduleIndex()
		self.due_rooms_ = []
		for room in self.database_.rooms_:
			schedule_char = room.room_scheduled_days[today_index]
			# Some cleaning required
			if (schedule_char != ""):
				# Today is a cleaning day
				if ((schedule_char == "x") or (schedule_char == "X")):
					# Cleaning method 2 --> Dry, Wet, Trash
					if (room.room_cleaning_method_ == 2):
						room.open_cleaning_tasks_.append(-1)
						room.open_cleaning_tasks_.append(0)
						room.open_cleaning_tasks_.append(1)
					# Cleaning method 1 --> Wet, Trash
					elif (room.room_cleaning_method_ == 1):
						room.open_cleaning_tasks_.append(-1)
						room.open_cleaning_tasks_.append(1)
					# Cleaning method 0 --> Dry, Trash
					elif (room.room_cleaning_method_ == 0)
						room.open_cleaning_tasks_.append(-1)
						room.open_cleaning_tasks_.append(0)
				# Today is only a trashcan day
				else:
					room.open_cleaning_tasks_.append(-1)
				# Append room to the due list
				self.due_rooms_.append(room)


	# Method for restoring the due list
	# CASE: Application stopped while not all rooms were completed. Restart --> Restore due list
	# USAGE: First scheduling to do
	def restoreDueRooms(self):
		for room in self.database_.rooms_:
			if (len(room.open_cleaning_tasks_) != 0):
				self.due_rooms_.append(room)


	# Method for extracting all overdue rooms from the due assignment
	# CASE: Application did not even start on a day it was supposed to.
	# USAGE: Run after all the due rooms are done
	def getAllOverdueRooms(self):
		# A room is overdue if
		# 1. The room is cleaned wet & the timestamp for wet cleaning is overdue
		# 2. The room is cleaned dry & the timestamp for dry cleaning is overdue
		# 3. The room is cleaned in both ways & one of the timestamps stated above is overdue
		# 4. The rooms trashcan timestamp is overdue
		# This method shifts back in time and checks each event for success

		today_index = 0
		current_schedule_index = today_index - 1
		day_delta = 1
		while (current_schedule_index != today_index):
			# Handle the case that today_index is a monday of an even week
			if (current_schedule_index == -1):
				current_schedule_index = 13
				continue
			# Get the corresponding date
			datetime_day_delta = datetime.timedelta(days=day_delta)
			indexed_date = date.today() - datetime_day_delta
			# Iterate through all potential rooms
			for room in self.database_.rooms_:
				# Room must not be in the due rooms list already
				if not (room in self.due_rooms_):
					schedule_char = room.room_scheduled_days_[current_schedule_index]
					cleaning_method = room.room_cleaning_method_
					# Room floor cleaning with trashcan was scheduled
					if (((schedule_char == "x") or (schedule_char == "X"))):
						dry_date = room.room_cleaning_timestamps_[1]
						wet_date = room.room_cleaning_timestamps_[2]
						# Room was to be cleaned dry and that did not happen
						if ((cleaning_method == 0) and (dry_date ! = None) and (dry_date < indexed_date)):
							room.open_cleaning_tasks_.append(0)
							if not (room in self.overdue_rooms_):
								self.overdue_rooms_.append(room)
						# Room was to be cleaned wet and that did not happen
						elif ((cleaning_method == 1) and (wet_date ! = None) and (wet_date < indexed_date)):
							room.open_cleaning_tasks_.append(1)
							if not (room in self.overdue_rooms_):
								self.overdue_rooms_.append(room)
						# Both cleanings were to be done and at least one did not happen
						elif (cleaning_method == 2):
							if ((dry_date != None) and (dry_date < indexed_date)):
								room.open_cleaning_tasks_.append(0)
								if not (room in self.overdue_rooms_):
									self.overdue_rooms_.append(room)
							if ((wet_date != None) and (wet_date < indexed_date)):
								room.open_cleaning_tasks_.append(0)
								if not (room in self.overdue_rooms_):
									self.overdue_rooms_.append(room)
					# Trashcan emptying was scheduled
					elif ((schedule_char == "p") or (schedule_char == "P")):
						trashcan_date = room.room_cleaning_timestamps_[0]
						if ((trashcan_date ! = None) and (trashcan_date >= indexed_date)):
							room.open_cleaning_tasks_.append(-1)
							if not (room in self.overdue_rooms_):
								self.overdue_rooms_.append(room)
					
			current_schedule_index = current_schedule_index - 1
			day_delta = day_delta + 1
			

	# Method for sorting a list of rooms after the cleaning method
	def sortRoomsList(self, rooms_list):
		rooms_wet_cleaning = []
		rooms_dry_cleaning = []
		for room in rooms_list:
			if (-1 in room.open_cleaning_tasks_):
				rooms_dry_cleaning.append(room)
			if (0 in room.open_cleaning_tasks_):
				rooms_dry_cleaning.append(room)
			if (1 in room.open_cleaning_tasks_):
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

