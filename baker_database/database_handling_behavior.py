#!/usr/bin/env python

import database
import database_classes
import datetime
from datetime import date


def getTodaysWeekType():
	weekNumber = date.today().isocalendar()[1]
	return weekNumber % 2

def getTodaysDayType():
	return date.today().weelday() 


class DatabaseHandler():
	database_ = None
	# The assignment which is due today
	due_assignment_ = None
	# Overdue assignments
	overdue_assignments_ = []
	# Contains all due rooms from the due assignments and removes them right after they were cleaned
	due_rooms_ = []
	# Contains all overdue rooms and removed them right after they were cleaned
	overdue_rooms_ = []

	def __init__(self, database):
		self.database_ = database

	# Method for extracting all due rooms from the due assignment
	def getAllDueAssignmentsAndRooms(self):

		# Get the due assignment
		for assignment in self.database_.assignments_:
			if ((assignment.assignment_week_type_ == getTodaysWeekType) 
			and (assignment.assignment_day_type_ == getTodaysDayType)):
				self.due_assignment_ = assignment
				break

		# If wanted: Get all overdue assignments
		if (self.database_.global_settings_.do_auto_complete_ == True):
			# Iterate through all assignments
			# Find those which are tagged to be unsuccessful
			pass

		# Get all rooms from the due assignment
		for room in self.due_assignment_.scheduled_rooms_data_:
			self.due_rooms_.append(room)

		# Remove all rooms which have been cleaned already
		# (If the assignment had been re- started half way, the completed rooms get removed)
		for room in self.due_rooms_:
			pass

		# If wanted: Get all overdue rooms
		# Remove those rooms which are already in the due_rooms_ list
		# Remove those rooms which have a good last_successful_clean_date_
		if (self.database_.global_settings_.do_auto_complete_ == True):
			for room in self.due_rooms_:
				if ((room in self.overdue_rooms_) 
				or False):
					self.overdue_rooms_.remove(room)

		# Tag all rooms to be unsuccessfully cleaned
		for room in self.due_rooms_:
			room.last_cleanup_successful_ = False
		for room in self.overdue_rooms_:
			room.last_cleanup_successful_ = False

		# Save all changes in database
		self.database_.saveDatabase()


	# Method for setting a room as completed
	def checkoutCompletedRoom(self, room, is_overdue):
		room.last_cleanup_successful_ = True
		room.last_successful_clean_date_ = datetime.datetime.now()
		if (is_overdue == True):
			for it_room in self.overdue_rooms_:
				if (it_room.room_id_ == room.room_id_):
					due_rooms_.remove(it_room)
		else:
			for it_room in self.due_rooms_:
				if (it_room.room_id_ == room.room_id_):
					due_rooms_.remove(it_room)
		# Save all changes to the database
		database_.saveDatabase()

	# Method for setting an overdue assignment as completed
	def checkoutAssignment(self, assignment, is_overdue):
		if (is_overdue == True):
			self.due_assignment_.
		else:
			pass

	# Method for checking if an overdue is completed
	def checkAssignmentCompletion(self, assignment):
		for room in assignment.scheduled_rooms_data_:
			pass
