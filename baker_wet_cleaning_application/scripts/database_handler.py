#!/usr/bin/env python

# For database
import database_classes
# For date and time calculations
from datetime import date, timedelta, datetime
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
	def isCleaningDay(schedule_char):
		return schedule_char == "x" or schedule_char == "X"

	@staticmethod
	def isTrashDay(schedule_char):
		return schedule_char == "p" or schedule_char == "P"

	@staticmethod
	def isDryCleaningMethod(cleaning_method):
		return cleaning_method == 2 or cleaning_method == 0

	@staticmethod
	def isWetCleaningMethod(cleaning_method):
		return cleaning_method == 2 or cleaning_method == 1

	@staticmethod
	def isTrashCleaningMethod(cleaning_method):
		return cleaning_method in [0, 1, 2]

	# ===============================================================================
	# OBJECT SPECIFIC METHODS
	# ===============================================================================

	def __init__(self, database):
		self.database_ = database

	def robotToday(self):
		return self.realToRobotDate(datetime.now())

	def getTodaysWeekType(self):
		weekNumber = self.robotToday().date().isocalendar()[1]
		return weekNumber % 2

	def getTodaysWeekDay(self):
		return self.robotToday().date().weekday()

	def getTodaysScheduleIndex(self):
		return self.getTodaysWeekType() * 7 + self.getTodaysWeekDay()

	def realToRobotDate(self, real_date):
		print("real_date:", real_date)
		print("self.database_.application_data_.planning_offset_:", self.database_.application_data_.planning_offset_)
		robot_date = real_date - timedelta(minutes=self.database_.application_data_.planning_offset_)
		return robot_date

	def robotToRealDate(self, robot_date):
		real_date = robot_date + timedelta(minutes=self.database_.application_data_.planning_offset_)
		return real_date

	# Get the room information in pixel
	def getMapAndRoomInformationInPixel(self, rooms_array):
		room_information_in_pixel = []
		bridge = CvBridge()
		segmentation_id = 0
		# Get the dimension of the image and create the temp map
		complete_map_opencv = bridge.imgmsg_to_cv2(self.database_.global_map_data_.map_image_, desired_encoding="passthrough")
		image_height, image_width = complete_map_opencv.shape
		tmp_map_opencv = np.zeros((image_height, image_width), np.uint8)
		for room in rooms_array:
			# Get an OPENCV representation of the image
			room_map_opencv = bridge.imgmsg_to_cv2(room.room_map_data_, desired_encoding="passthrough")
			# Add the room to the final map
			for x in range(image_width):
				for y in range(image_height):
					if room_map_opencv[y, x] == 255:
						tmp_map_opencv[y, x] = segmentation_id + 1
			# Get room_information_in_pixels
			room_information_in_pixel.append(room.room_information_in_pixel_)
			segmentation_id = segmentation_id + 1
		segmented_map = bridge.cv2_to_imgmsg(tmp_map_opencv, encoding="mono8")
		return room_information_in_pixel, segmented_map

	# Get the room information in meter
	def getRoomInformationInMeter(self, rooms_array):
		room_information_in_meter = {}
		for room in rooms_array:
			room_id = room.room_id_
			room_information_in_meter[room_id] = room.room_information_in_meter_
		return room_information_in_meter

	@staticmethod
	# Create a mapping RoomSequenceResult |-> RoomObject
	def getRoomMapping(rooms_list, room_sequence_result):
		mapping = []
		for checkpoint in room_sequence_result.checkpoints:
			for current_room_index in checkpoint.room_indices:
				mapping.append(rooms_list[current_room_index].room_id_)
		return mapping

	# Method for extracting all due rooms from the due assignment
	# CASE: First run of application, no rooms collected yet today.
	# USAGE: Run when the application is started the first time today. Then run at the beginning.
	def computeAllDueRooms(self):
		print "[DatabaseHandler]: computing all due rooms ..."

		self.due_rooms_ = []
		self.restoreDueRooms()

		# If the application ran already today and the due rooms list is not empty, this should not run
		last_planning_dates = self.database_.application_data_.last_planning_date_
		today = self.robotToday().date()

		if last_planning_dates[0] is not None and today == self.realToRobotDate(last_planning_dates[0]).date()\
			and len(self.due_rooms_) != 0:
			print "[DatabaseHandler]: Earlier run detected!"
			return False

		today_index = self.getTodaysScheduleIndex()

		for room in self.database_.rooms_:
			cleaning_tasks = set(room.open_cleaning_tasks_)
			method = room.room_cleaning_method_
			schedule_char = room.room_scheduled_days_[today_index]
			# Some cleaning required
			if schedule_char == "":
				continue

			# Find out if the timestamps indicate that the room has been handled already today
			date_stamps = room.room_cleaning_datestamps_

			already_done = {
				'TRASH': date_stamps[0] is not None and self.realToRobotDate(date_stamps[0]).date() == today,
				'DRY': date_stamps[1] is not None and self.realToRobotDate(date_stamps[1]).date() == today,
				'WET': date_stamps[2] is not None and self.realToRobotDate(date_stamps[2]).date() == today
			}

			# If today is a cleaning day
			if self.isCleaningDay(schedule_char):

				if self.isTrashCleaningMethod(method) and not already_done['TRASH']:
					cleaning_tasks.add(self.TRASH_TASK)

				if self.isDryCleaningMethod(method) and not already_done['DRY']:
					cleaning_tasks.add(self.DRY_TASK)

				if self.isWetCleaningMethod(method) and not already_done['WET']:
					cleaning_tasks.add(self.WET_TASK)

			# If today is only a trashcan day
			elif self.isTrashDay(schedule_char):
				cleaning_tasks.add(self.TRASH_TASK)

			room.open_cleaning_tasks_ = list(cleaning_tasks)
			# Append room to the due list if any task is to be done
			if len(room.open_cleaning_tasks_) != 0:
				self.due_rooms_.append(room)

		self.applyChangesToDatabase()  # saves all the due rooms in the database
		return True

	# Method for restoring the due list after application was stopped
	# CASE: Application stopped while not all rooms were completed, but room collecting completed. Restart --> Restore due list
	# USAGE: Run at the beginning of computeAllDueRooms
	def restoreDueRooms(self):
		print "[DatabaseHandler]: Restoring due rooms from earlier runs..."
		self.due_rooms_ = []
		for room in self.database_.rooms_:
			if len(room.open_cleaning_tasks_) != 0:
				self.due_rooms_.append(room)

	# Method for extracting all overdue rooms from the due assignment
	# CASE: Some cleaning subtasks were not completed in the past (i.e. a scheduled one was missed)
	# USAGE: Run after all the due rooms are done
	def computeAllOverdueRooms(self):
		print("[DatabaseHandler] computeAllOverdueRooms")
		today_index = self.getTodaysScheduleIndex()
		today = self.robotToday()

		overdue_rooms = set()

		for room in self.database_.rooms_:
			if room in self.due_rooms_:
				continue

			cleaning_tasks = set(room.open_cleaning_tasks_)
			for day_delta in range(1, 14):
				current_schedule_index = (today_index - day_delta) % 14
				current_schedule_date = today - timedelta(days=day_delta)

				schedule_char = room.room_scheduled_days_[current_schedule_index]
				cleaning_method = room.room_cleaning_method_
				# Room floor cleaning with trashcan was scheduled
				if self.isCleaningDay(schedule_char):
					trashcan_date = self.realToRobotDate(room.room_cleaning_datestamps_[0])
					dry_date = self.realToRobotDate(room.room_cleaning_datestamps_[1])
					wet_date = self.realToRobotDate(room.room_cleaning_datestamps_[2])
					# Room's trashcan was to be emptied and that did not happen
					if self.isTrashCleaningMethod(cleaning_method) and (trashcan_date is None or trashcan_date < current_schedule_date):
						cleaning_tasks.add(self.TRASH_TASK)
						overdue_rooms.add(room)

					# Room was to be cleaned dry and that did not happen
					if self.isDryCleaningMethod(cleaning_method) and (dry_date is None or dry_date < current_schedule_date):
						cleaning_tasks.add(self.DRY_TASK)
						overdue_rooms.add(room)

					# Room was to be cleaned wet and that did not happen
					if self.isWetCleaningMethod(cleaning_method) and (wet_date is None or wet_date < current_schedule_date):
						cleaning_tasks.add(self.WET_TASK)
						overdue_rooms.add(room)

				# Trashcan emptying was scheduled
				elif self.isTrashDay(schedule_char):
					trashcan_date = room.room_cleaning_datestamps_[0]
					if trashcan_date is None or trashcan_date < current_schedule_date:
						cleaning_tasks.add(self.TRASH_TASK)
						overdue_rooms.add(room)

			room.open_cleaning_tasks_ = list(cleaning_tasks)

		self.overdue_rooms_ = list(overdue_rooms)

	# Method for figuring out whether the application had been started today already
	def noPlanningHappenedToday(self):
		last_start = self.realToRobotDate(self.database_.application_data_.last_planning_date_[0])
		today_date = self.robotToday()
		if last_start is not None:
			delta = today_date - last_start
			return delta.days >= 1
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
		log_item.date_and_time_ = datetime.now()
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
		room.room_cleaning_datestamps_[assignment_type + 1] = datetime.now()

		# Save all changes to the database
		self.applyChangesToDatabase()

	# Public method to add an entry to the log. Method from the database does not need to be called, avoiding nasty imports
	def addLogEntry(self, room_id, status, cleaning_task, found_dirtspots, found_trashcans, cleaned_surface_area,
					room_issues, used_water_amount, battery_usage):
		new_entry = database_classes.LogItem()
		new_entry.room_id_ = room_id
		new_entry.log_week_and_day_ = [self.getTodaysWeekType(), self.getTodaysWeekDay()]
		new_entry.date_and_time_ = datetime.now()
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
