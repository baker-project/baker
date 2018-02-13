#!/usr/bin/env python

import behavior_container
import database
import database_classes
import datetime

class DatabaseHandlingBehavior(behavior_container.BehaviorContainer):
	database_ = None
	# Contains all due assignments
	due_assignments_ = []
	# Contains all due rooms from the due assignments and removes them right after they were cleaned
	due_rooms_ = []

	def __init__(self, database):
		self.database_ = database

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		self.database_.saveDatabase()
		# undo or check whether everything has been undone

	# Method for setting parameters for the behavior
	def setParameters(self):
		pass

	# Method for extracting all due assignments and all due rooms
	def updateAllDueAssignmentsAndRooms(self):
		for assignment in self.database_.assignments_:
			# Check if there was an explicit date the assignment had to be done
			# Append all corresponding rooms to the fringe
			explicit_date_relevant = False
			for date in assignment.clean_dates_:
				if (date < datetime.datetime.now()):
					explicit_date_relevant = True
			if (explicit_date_relevant == True):
				self.due_assignments_.append(self.database_.getAssignment(assignment.assignment_id_))
				for room in assignment.rooms_data: 
					room.last_cleanup_successful = False
					self.due_rooms_.append(room)
			# Check if the last success date within the assignment is too far in the past
			# Append all corresponding rooms if their own date is also too far in the past
			elif (assignment.last_completed_clean_ + assignment.clean_interval_ < datetime.datetime.now()):
				self.due_assignments_.append(self.database_.getAssignment(assignment.assignment_id_))
				for room in assignment.rooms_data:
					if (room.last_successful_clean_date_ + assignment.clean_interval_ < datetime.datetime.now()):
						room.last_cleanup_successful = False
						self.due_rooms_.append(room)
		# Check if there are duplicate rooms and remove them from the due list
		self.due_rooms_ = set(due_rooms_)
		# Save all changes to the database
		database_.saveDatabase()

	# Method for setting a room as completed
	def checkoutCompletedRoom(self, room):
		room.last_cleanup_successful_ = True
		room.last_successful_clean_date_ = datetime.datetime.now()
		for it_room in self.due_rooms_:
			if (it_room.room_id_ == room.room_id_):
				due_rooms_.remove(it_room)
		# Save all changes to the database
		database_.saveDatabase()

	# Method for checking if all assignments are completed
	def handleFinishedTask(self):
		for assignment in self.due_assignments_:
			# Check if all rooms of the assignment are completed
			all_rooms_completed = True
			for room in assignment.scheduled_rooms_:
				for due_room in self.due_rooms_:
					if (room.room_id_ == due_room.room_id_):
						all_rooms_completed = False
			# If so, update the date, remove old explicit dates, remove the assignment from the due list
			if (all_rooms_completed == True):
				for date in assignment.clean_dates_:
					if (date < datetime.datetime.now()):
						assignment.clean_dates_.remove(date)
				assignment.last_completed_clean_ = datetime.datetime.now()
				self.due_assignments_.remove(assignment)
		# Save all changes to the database
		database_.saveDatabase()


	# Implemented Behavior
	def executeCustomBehavior(self):
		# After each command, def handleInterrupt has to be executed:
		# if self.handleInterrupt() == 2:
		# return
		pass
		
				
		


'''
How to call the implemented behavior:

xyz = database_handling_behavior.DatabaseHandlingBehavior("DatabaseHandlingBehavior", self.interrupt_var_)

...

xyz.executeBehavior()
'''