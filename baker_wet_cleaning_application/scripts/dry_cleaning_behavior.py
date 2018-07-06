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
	# Services to be used:
	# NONE
	#========================================================================

	def __init__(self, behavior_name, interrupt_var):
		self.behavior_name_ = behavior_name
		self.interrupt_var_ = interrupt_var
		
	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, sequencing_result, mapping):
		self.database_handler_ = database_handler
		self.sequencing_result_ = sequencing_result
		self.mapping_ = mapping

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Searching for trashcans
	def trashcanRoutine(self):
		pass

	# Searching for dirt
	def dirtRoutine(self):
		pass

	# Driving through room
	def exploreRoom(self):
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		self.tool_changer_ = tool_changing_behavior.ToolChangingBehavior("ToolChangingBehavior", self.interrupt_var_)
		self.trolley_mover_ = trolley_movement_behavior.TrolleyMovementBehavior("TrolleyMovingBehavior", self.interrupt_var_)

		# TOOL CHANGE ACCORDING TO CLEANING TASK
		# ======================================
		self.tool_changer_.setParameters(self.database_handler_)
		self.tool_changer_.executeBehavior()

		for checkpoint in self.sequencing_result_.checkpoints:

			# TROLLEY MOVEMENT TO CHECKPOINT
			# ==============================
			self.trolley_mover_.setParameters(self.database_handler_)
			self.trolley_mover_.executeBehavior()

			room_counter = 0

			for room_index in checkpoint.room_indices:

				# HANDLING OF SELECTED ROOM
				# =========================
				thread = threading.Thread(target = self.exploreRoom)
				thread.start()
				thread = threading.Thread(target = self.dirtRoutine)
				thread.start()
				thread = threading.Thread(target = self.trashcanRoutine)
				thread.start()
				
				# Checkout the completed the room
				self.printMsg("ID of cleaned room: " + str(self.mapping_.get(room_counter)))
				self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), 1)
				self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), -1)
				self.printMsg(str(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)).open_cleaning_tasks_))

				room_counter = room_counter + 1
				
