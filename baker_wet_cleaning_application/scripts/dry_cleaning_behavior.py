#!/usr/bin/env python

import threading
import time
import rospy
import behavior_container
import database
import tool_changing_behavior
import trolley_movement_behavior


class DryCleaningBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Handles the dry cleanig process (i.e. Exploring, Dirt removal, Trashcan)
	# for all rooms provided in a given list
	#========================================================================

	def __init__(self, behavior_name, interrupt_var):
		self.behavior_name_ = behavior_name
		self.interrupt_var_ = interrupt_var
		
	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, sequencing_result, mapping):
		self.database_handler_ = database_handler
		self.sequencing_result_ = sequencing_result
		self.mapping_ = mapping

	# Method for returning to the standard state of the robot
	def returnToRobotStandardState(self):
		# nothing to be saved
		# nothing to be undone
		pass

	# Searching for trashcans
	def trashcanRoutine(self, room_counter):
		# ==========================================
		# insert trashcan handling here
		# ==========================================
		self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), -1)

		# Adding log entry for wet cleaning
		self.database_handler_.addLogEntry(
			self.mapping_.get(room_counter), # room id
			1, # status (1=Completed)
			-1, # cleaning task (-1=trashcan only)
			0, # (found dirtspots)
			0, # trashcan count
			0, # surface area
			[], # room issues
			0, # water amount
			0 # battery usage
		)

	# Searching for dirt
	def dirtRoutine(self, room_counter):
		# ==========================================
		# insert dirt removing here
		# ==========================================
		self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), 0)

	# Driving through room
	def exploreRoom(self, room_counter):
		# ==========================================
		# insert room exploration here
		# ==========================================
		
		# Adding log entry for wet cleaning
		self.database_handler_.addLogEntry(
			self.mapping_.get(room_counter), # room id
			1, # status (1=Completed)
			0, # cleaning task (0=dry only)
			0, # (found dirtspots)
			0, # trashcan count
			0, # surface area
			[], # room issues
			0, # water amount
			0 # battery usage
		)

	# Implemented Behavior
	def executeCustomBehavior(self):
		self.tool_changer_ = tool_changing_behavior.ToolChangingBehavior("ToolChangingBehavior", self.interrupt_var_)
		self.trolley_mover_ = trolley_movement_behavior.TrolleyMovementBehavior("TrolleyMovingBehavior", self.interrupt_var_)

		# Tool change according to cleaning task
		self.tool_changer_.setParameters(self.database_handler_)
		self.tool_changer_.executeBehavior()

		room_counter = 0

		for checkpoint in self.sequencing_result_.checkpoints:

			# Trolley movement to checkpoint
			self.trolley_mover_.setParameters(self.database_handler_)
			self.trolley_mover_.executeBehavior()

			for room_index in checkpoint.room_indices:

				# Handling of selected room
				cleaning_tasks = self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)).open_cleaning_tasks_
				exploring_thread = threading.Thread(target = self.exploreRoom(room_counter))
				exploring_thread.start()
				if ((0 in cleaning_tasks) == True):
					dirt_thread = threading.Thread(target = self.dirtRoutine(room_counter))
					dirt_thread.start()
				if ((-1 in cleaning_tasks) == True):
					trashcan_thread = threading.Thread(target = self.trashcanRoutine(room_counter))
					trashcan_thread.start()
				exploring_thread.join()
				
				# Checkout the completed room
				self.printMsg("ID of dry cleaned room: " + str(self.mapping_.get(room_counter)))
				self.printMsg(str(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)).open_cleaning_tasks_))
				room_counter = room_counter + 1
				
