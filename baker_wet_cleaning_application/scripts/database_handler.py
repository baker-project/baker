#!/usr/bin/env python

# For database
import database_classes
# For date and time calculations
import datetime
from datetime import date
# For room information
from ipa_building_msgs.msg import *
# For map generation
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class DatabaseHandler:

	#========================================================================
	# Description:
	# Class for evaluating a Database object and adding/removing information.
	#========================================================================

	database_ = None
	# Contains all due rooms
	due_rooms_ = []
	# Contains all overdue rooms
	overdue_rooms_ = []

	TRASH_TASK = -1
	DRY_TASK = 0
	WET_TASK = 1

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
	def isCleaningDay(schedule_char):
		return schedule_char == "x" or schedule_char == "X"

	@staticmethod
	def isTrashDay(schedule_char):
		return schedule_char == "p" or schedule_char == "P"

	@staticmethod
	def isDryDay(cleaning_method):
		return cleaning_method == 2 or cleaning_method == 0

	@staticmethod
	def isWetDay(cleaning_method):
		return cleaning_method == 2 or cleaning_method == 1

	@staticmethod
	def isTrashDay(cleaning_method):
		return cleaning_method in [0, 1, 2]
	# ===============================================================================
	# OBJECT SPECIFIC METHODS
	# ===============================================================================

	# Get the room information in pixel
	def getMapAndRoomInformationInPixel(self, rooms_array):
		room_information_in_pixel = []
		bridge = CvBridge()
		segmentation_id = 0
		# Get the dimension of the image and create the temp map
		complete_map_opencv = bridge.imgmsg_to_cv2(self.database_.global_map_data_.map_image_, desired_encoding = "passthrough")
		image_height, image_width = complete_map_opencv.shape
		tmp_map_opencv = np.zeros((image_height, image_width), np.uint8)
		for room in rooms_array:
			# Get an OPENCV representation of the image
			room_map_opencv = bridge.imgmsg_to_cv2(room.room_map_data_, desired_encoding = "passthrough")
			# Add the room to the final map
			for x in range(image_width):
				for y in range(image_height):
					if room_map_opencv[y, x] == 255:
						tmp_map_opencv[y, x] = segmentation_id + 1
			# Get room_information_in_pixels
			room_information_in_pixel.append(room.room_information_in_pixel_)
			segmentation_id = segmentation_id + 1
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


	# Create a mapping RoomSequenceResult |-> RoomObject
	def getRoomMapping(self, rooms_list, room_sequence_result):
		mapping = {}
		rooms_index = 0
		for checkpoint in room_sequence_result.checkpoints:
			for current_room_index in checkpoint.room_indices:
				mapping[rooms_index] = rooms_list[current_room_index].room_id_
				rooms_index = rooms_index + 1
		return mapping


	# Method for extracting all due rooms from the due assignment
	# CASE: First run of application, no rooms collected yet today.
	# USAGE: Run when the application is started the first time today. Then run at the beginning.
	def getAllDueRooms(self):
		print "[DatabaseHandler]: getAllDueRooms() ..."

		# If the application ran already today and the due rooms list is not empty, this should not run
		if self.database_.application_data_.last_planning_date_[0] is not None:
			delta = datetime.datetime.now() - self.database_.application_data_.last_planning_date_[0]
			if delta.days == 0 and len(self.due_rooms_) != 0:
				print "[DatabaseHandler]: Earlier run detected!"
				return

		today_index = self.getTodaysScheduleIndex()
		self.due_rooms_ = []
		for room in self.database_.rooms_:
			schedule_char = room.room_scheduled_days_[today_index]
			# Some cleaning required
			if schedule_char != "":
				# Find out if the timestamps indicate that the room has been handled already today
				timestamp_is_new = [
					room.room_cleaning_datestamps_[0] is not None
					and datetime.datetime.now() - room.room_cleaning_datestamps_[0] < datetime.timedelta(days=1),	# trash cans
					room.room_cleaning_datestamps_[1] is not None
					and datetime.datetime.now() - room.room_cleaning_datestamps_[1] < datetime.timedelta(days=1),	# dry cleaning
					room.room_cleaning_datestamps_[2] is not None
					and datetime.datetime.now() - room.room_cleaning_datestamps_[2] < datetime.timedelta(days=1)	# wet cleaning
				]

				# If today is a cleaning day
				if self.isCleaningDay(schedule_char):

					if self.isTrashDay(room.room_cleaning_method_)\
						and (not timestamp_is_new[0] and not (self.TRASH_TASK in room.open_cleaning_tasks_)):
							room.open_cleaning_tasks_.append(self.TRASH_TASK)

					if self.isDryDay(room.room_cleaning_method_)\
						and (not timestamp_is_new[1] and not (self.DRY_TASK in room.open_cleaning_tasks_)):
							room.open_cleaning_tasks_.append(self.DRY_TASK)

					if self.isWetDay(room.room_cleaning_method_)\
						and (not timestamp_is_new[2] and not (self.WET_TASK in room.open_cleaning_tasks_)):
							room.open_cleaning_tasks_.append(self.WET_TASK)

				# If today is only a trashcan day
				else:	# todo: check for "p" since trash is not just standard procedure
					room.open_cleaning_tasks_.append(self.TRASH_TASK)

				# Append room to the due list if any task is to be done
				if len(room.open_cleaning_tasks_) != 0:
					self.due_rooms_.append(room)
		
		self.applyChangesToDatabase() # saves all the due rooms in the database


	# Method for restoring the due list after application was stopped
	# CASE: Application stopped while not all rooms were completed, but room collecting completed. Restart --> Restore due list
	# USAGE: Run before getAllDueRooms()
	def restoreDueRooms(self):
		print "[DatabaseHandler]: Restoring due rooms from earlier runs..."
		for room in self.database_.rooms_:
			if len(room.open_cleaning_tasks_) != 0:
				self.due_rooms_.append(room)


	# Method for extracting all overdue rooms from the due assignment
	# CASE: Some cleaning subtasks were not completed in the past (i.e. a scheduled one was missed)
	# USAGE: Run after all the due rooms are done
	def getAllOverdueRooms(self):
		self.getAllOverdueRoomsAmongRooms(self.database_.rooms_)
		self.applyChangesToDatabase()

	@staticmethod
	def getAllOverdueRoomsAmongRooms(self, rooms):
		today_index = 0		# todo: verify whether this is always correct: todo_index should be determined correctly for the given day
		current_schedule_index = today_index - 1
		day_delta = 1
		while current_schedule_index != today_index:
			# Handle the case that today_index is a monday of an even week
			if current_schedule_index == -1:
				current_schedule_index = 13
				continue
			# Get the corresponding date
			datetime_day_delta = datetime.timedelta(days=day_delta)
			indexed_date = datetime.datetime.now() - datetime_day_delta
			# Iterate through all potential rooms
			for room in rooms:
				# Room must not be in the due rooms list already
				if not (room in self.due_rooms_):
					schedule_char = room.room_scheduled_days_[current_schedule_index]
					cleaning_method = room.room_cleaning_method_
					# Room floor cleaning with trashcan was scheduled
					if self.isCleaningDay(schedule_char):
						trashcan_date = room.room_cleaning_datestamps_[0]
						dry_date = room.room_cleaning_datestamps_[1]
						wet_date = room.room_cleaning_datestamps_[2]

						# Room's trashcan was to be emptied and that did not happen
						if trashcan_date is None and trashcan_date < indexed_date:
							if not (self.TRASH_TASK in room.open_cleaning_tasks_):
								room.open_cleaning_tasks_.append(self.TRASH_TASK)
							if not (room in self.overdue_rooms_):
								self.overdue_rooms_.append(room)

						# Room was to be cleaned dry and that did not happen
						if self.isDryDay(cleaning_method) and dry_date is not None and dry_date < indexed_date:
							if not (self.DRY_TASK in room.open_cleaning_tasks_):
								room.open_cleaning_tasks_.append(self.DRY_TASK)
							if not (room in self.overdue_rooms_):
								self.overdue_rooms_.append(room)

						# Room was to be cleaned wet and that did not happen
						if self.isWetDay(cleaning_method) and wet_date is not None and wet_date < indexed_date:
							if not (self.WET_TASK in room.open_cleaning_tasks_):
								room.open_cleaning_tasks_.append(self.WET_TASK)
							if not (room in self.overdue_rooms_):
								self.overdue_rooms_.append(room)

					# Trashcan emptying was scheduled
					elif self.isTrashDay(cleaning_method):
						trashcan_date = room.room_cleaning_datestamps_[0]
						if trashcan_date is not None and trashcan_date < indexed_date:
							if not (self.TRASH_TASK in room.open_cleaning_tasks_):
								room.open_cleaning_tasks_.append(-1)
							if not (room in self.overdue_rooms_):
								self.overdue_rooms_.append(room)
					
			current_schedule_index = current_schedule_index - 1
			day_delta = day_delta + 1


	# Method for figuring out whether the application had been started today already
	def noPlanningHappenedToday(self):
		last_start = self.database_.application_data_.last_planning_date_[0]
		today_date = datetime.datetime.now()
		if last_start is not None:
			delta = today_date - last_start
			if delta.days < 1:
				return False
			else:
				return True
		else:
			return True


	# Method for sorting a list of rooms after the cleaning method
	def sortRoomsList(self, rooms_list):
		rooms_wet_cleaning = []
		rooms_dry_cleaning = []
		# Divide between wet and dry cleaning 
		for room in rooms_list:
			if self.DRY_TASK in room.open_cleaning_tasks_:
				rooms_dry_cleaning.append(room)
			if self.WET_TASK in room.open_cleaning_tasks_:
				rooms_wet_cleaning.append(room)

		# Append all trashcan-only-rooms to the dry
		for room in rooms_list:
			if (self.TRASH_TASK in room.open_cleaning_tasks_) and len(room.open_cleaning_tasks_) == 1:
				rooms_dry_cleaning.append(room)

		return rooms_dry_cleaning, rooms_wet_cleaning


	# Method for setting a room as completed
	def checkoutCompletedRoom(self, room, assignment_type):
		# Print checkout on console
		print "[DatabaseHandler]: Checking out room " + str(room.room_id_) + ", cleaning subtask " + str(assignment_type)\
			  + ", out of " + str(room.open_cleaning_tasks_)
		# Add entry into the log
		log_item = database_classes.LogItem()
		log_item.room_id_ = room.room_id_
		log_item.cleaning_task_ = assignment_type
		log_item.battery_usage_ = 0
		log_item.cleaned_surface_area_ = 0
		log_item.date_and_time_ = datetime.datetime.now()
		log_item.found_dirtspots_ = 0
		log_item.found_trashcans_ = 0
		log_item.log_week_and_day_ = [self.getTodaysWeekType(), self.getTodaysWeekDay()]
		log_item.room_issues_ = []
		log_item.status_ = 0
		log_item.trolley_capacity_ = 0
		log_item.used_water_amount_ = 0
		self.database_.addLogEntry(log_item)
		# Remove assignment from the room's open assignment list
		room.open_cleaning_tasks_.remove(assignment_type)
		# Save current datetime as timestamp for the specified assignment
		room.room_cleaning_datestamps_[assignment_type + 1] = datetime.datetime.now()
		# Save all changes to the database
		self.applyChangesToDatabase()

	
	# Public method to add an entry to the log. Method from the database does not need to be called, avoiding nasty imports
	def addLogEntry(self, room_id, status, cleaning_task, found_dirtspots, found_trashcans, cleaned_surface_area,
					room_issues, used_water_amount, battery_usage):
		new_entry = database_classes.LogItem()
		new_entry.room_id_ = room_id
		new_entry.log_week_and_day_ = [self.getTodaysWeekType(), self.getTodaysWeekDay()]
		new_entry.date_and_time_ = datetime.datetime.now()
		new_entry.status_ = status
		new_entry.cleaning_task_ = cleaning_task
		new_entry.found_dirtspots_ = found_dirtspots
		new_entry.found_trashcans_ = found_trashcans
		new_entry.cleaned_surface_area_ = cleaned_surface_area
		new_entry.room_issues_ = room_issues
		new_entry.used_water_amount_ = used_water_amount
		new_entry.battery_usage_ = battery_usage
		self.database_.addLogEntry(new_entry)

	# Method to run if a change in the database shall be applied (i.e. writes the temporary files). Applied changes can be discarded
	def applyChangesToDatabase(self):
		self.database_.saveCompleteDatabase(temporal_file=True)

	# Method to run after all cleaning operations were performed
	def cleaningFinished(self):
		self.database_.saveCompleteDatabase(temporal_file=False)

