#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D, Point32, Quaternion
import std_srvs.srv
import dynamic_reconfigure.client
import ipa_building_msgs.srv

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import threading

import behavior_container
import move_base_behavior
import room_exploration_behavior
import move_base_path_behavior
import trolley_movement_behavior
import trashcan_emptying_behavior
import tool_changing_behavior
import move_base_wall_follow_behavior
import room_wet_floor_cleaning_behavior

class WetCleaningBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Handles the wet cleanig process (i.e. Floor cleaning, Trashcan)
	# for all rooms provided in a given list
	#========================================================================

		
	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, room_information_in_meter, sequence_data, mapping, robot_frame_id, robot_radius, coverage_radius, field_of_view, field_of_view_origin, use_cleaning_device):
		# Parameters set from the outside
		self.database_handler_= database_handler
		self.room_information_in_meter_ = room_information_in_meter
		self.sequence_data_ = sequence_data
		self.mapping_ = mapping
		self.robot_frame_id_ = robot_frame_id
		self.robot_radius_ = robot_radius
		self.coverage_radius_ = coverage_radius
		self.field_of_view_ = field_of_view
		self.field_of_view_origin_ = field_of_view_origin
		self.use_cleaning_device_ = use_cleaning_device	# todo: hack: cleaning device can be turned off for trade fair show
		# Parameters set autonomously
		self.room_exploration_service_str_ = '/room_exploration/room_exploration_server'
		self.move_base_path_service_str_ = '/move_base_path'
		self.move_base_wall_follow_service_str_ = '/move_base_wall_follow'
		self.move_base_service_str_ = 'move_base'
		self.start_cleaning_service_str_ = '/brush_cleaning_module_interface/start_brush_cleaner'
		self.stop_cleaning_service_str_ = '/brush_cleaning_module_interface/stop_brush_cleaner'
		self.coverage_monitor_dynamic_reconfigure_service_str_ = '/room_exploration/coverage_monitor_server'
		self.stop_coverage_monitoring_service_str_ = "/room_exploration/coverage_monitor_server/stop_coverage_monitoring"
		self.receive_coverage_image_service_str_ = "/room_exploration/coverage_monitor_server/get_coverage_image"
		self.trolley_movement_service_str_ = ""
		self.tool_changing_service_str_ = ""



	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# nothing to save
		# nothing to be undone
		pass



	# Driving through room and wet cleaning
	def driveCleaningTrajectory(self, room_counter, current_room_index):

		self.printMsg("Moving to next room with current_room_index = " + str(current_room_index))

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		self.room_wet_floor_cleaner_.setParameters(
			self.database_handler_.database_.getRoom(self.mapping_.get(current_room_index)).room_map_data_, 
			self.room_information_in_meter_[current_room_index].room_center,
			self.database_handler_.database_.global_map_data_.map_image_, 
			self.database_handler_.database_.global_map_data_.map_resolution_, 
			self.database_handler_.database_.global_map_data_.map_origin_, 
			self.database_handler_.database_.global_map_data_.map_header_frame_id_, 
			self.robot_frame_id_, 
			self.robot_radius_, 
			self.coverage_radius_, 
			self.field_of_view_,
			self.field_of_view_origin_,
			self.use_cleaning_device_	# todo: hack: cleaning device can be turned off for trade fair show
		)
		self.room_wet_floor_cleaner_.executeBehavior()

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Mark the current room as finished
		self.printMsg("ID of cleaned room: " + str(self.mapping_.get(room_counter)))
		self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), 1)
		self.printMsg(str(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)).open_cleaning_tasks_))

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Adding log entry for wet cleaning
		self.database_handler_.addLogEntry(
			self.mapping_.get(room_counter), # room id
			1, # status (1=Completed)
			1, # cleaning task (1=wet only)
			0, # (found dirtspots)
			0, # trashcan count
			0, # surface area
			[], # room issues
			0, # water amount
			0 # battery usage
		)



	# Searching for trashcans
	def trashcanRoutine(self, room_counter):
		# ==========================================
		# insert trashcan handling here
		# ==========================================
		self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), -1)

		# Adding log entry for trashcan emptying
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



	# Implemented Behavior
	def executeCustomBehavior(self):
		self.trolley_mover_ = trolley_movement_behavior.TrolleyMovementBehavior("TrolleyMovementBehavior", self.interrupt_var_)
		self.tool_changer_ = tool_changing_behavior.ToolChangingBehavior("ToolChangingBehavior", self.interrupt_var_)
		self.room_wet_floor_cleaner_ = room_wet_floor_cleaning_behavior.RoomWetFloorCleaningBehavior("RoomWetFloorCleaningBehavior", self.interrupt_var_)
		

		# Tool changing
		self.tool_changer_.setParameters(self.database_handler_)
		self.tool_changer_.executeBehavior()

		# Room counter index: Needed for mapping of room_indices <--> RoomItem.room_id
		room_counter = 0

		for current_checkpoint_index in range(len(self.sequence_data_.checkpoints)):

			# Trolley movement to checkpoint
			self.trolley_mover_.setParameters(self.database_handler_)
			self.trolley_mover_.executeBehavior()

			for current_room_index in self.sequence_data_.checkpoints[current_checkpoint_index].room_indices:

				# Handling of selected room
				cleaning_thread = threading.Thread(target = self.driveCleaningTrajectory(room_counter, current_room_index))
				cleaning_thread.start()
				cleaning_tasks = self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)).open_cleaning_tasks_
				if ((-1 in cleaning_tasks) == True):
					trashcan_thread = threading.Thread(target = self.trashcanRoutine(room_counter))
					trashcan_thread.start()
				cleaning_thread.join()
				
				# Increment the current room counter index
				room_counter = room_counter + 1