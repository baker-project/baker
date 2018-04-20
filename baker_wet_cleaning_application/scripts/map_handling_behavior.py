#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import behavior_container
import map_receiving_behavior
import map_segmentation_behavior
import room_sequencing_behavior
import map_segment_extracting_behavior

class MapHandlingBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Serivces to be used:
	# map_receiving_service_str_:
	#       '/baker/get_map_image'
	# map_segmentation_service_str_:
	#       '/room_segmentation/room_segmentation_server'
	# room_sequencing_service_str_ = 
	#       '/room_sequence_planning/room_sequence_planning_server'
	#========================================================================
	
	# Method for returning to the standard pose of the robot
	def setParameters(self, robot_radius, database_handler, is_wet=False, is_overdue=False):
		# Parameters set autonomously
		self.map_receiving_service_str_ = '/map_management_client/get_map_image'
		self.map_segmented_receiving_service_str_ = '/map_management_client/get_map_segmented_image'
		self.map_segmentation_service_str_ = '/room_segmentation/room_segmentation_server'
		self.room_sequencing_service_str_ = '/room_sequence_planning/room_sequence_planning_server'
		# Parameters set from the outside
		self.database_handler_ = database_handler
		self.is_wet_ = is_wet
		self.is_overdue_ = is_overdue
		# DEPRECATED, use database in future
		self.robot_radius_ = robot_radius


	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no data to be saved
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):

		"""
		# DEPRECATED MAP RECEIVING ROUTINE

		# Map receiving
		self.map_receiver_ = map_receiving_behavior.MapReceivingBehavior("Map receiving", self.interrupt_var_, self.map_receiving_service_str_, self.map_segmented_receiving_service_str_)
		self.map_receiver_.setParameters()
		self.map_receiver_.executeCustomBehavior()
		self.map_data_ = self.map_receiver_.map_data_
		self.printMsg(self.map_data_.map.header.frame_id)


		# DEPRECATED SEGMENTATION ROUTINE

		self.map_segmented_data_ = self.map_receiver_.map_segmented_data_

		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		
		# Map segmentation
		map_data = self.map_data_
		map_segmentation_algorithm = 0		# use standard pre-set method
		if (self.map_segmented_data_!=None):
			self.printMsg("Using the externally pre-segmented map.")
			map_data = self.map_segmented_data_
			map_segmentation_algorithm = 99		# set segmentation method passthrough
		self.map_segmenter_ = map_segmentation_behavior.MapSegmentationBehavior("Map segmentation", self.interrupt_var_, self.map_segmentation_service_str_)
		self.map_segmenter_.setParameters(
			map_data,
			self.robot_radius_,
			map_segmentation_algorithm
			)
		self.map_segmenter_.executeCustomBehavior()
		self.segmentation_data_ = self.map_segmenter_.segmentation_result_
		"""
		

		# Extract all rooms out of the database which are relevant
		if (self.is_overdue_ == False):
			self.database_handler_.getAllDueRooms()
			rooms_trashcan_only, rooms_dry_cleaning, rooms_wet_cleaning = self.database_handler_.sortRoomsList(self.database_handler_.due_rooms_)
		else:
			self.database_handler_.getAllOverdueRooms()
			rooms_trashcan_only, rooms_dry_cleaning, rooms_wet_cleaning = self.database_handler_.sortRoomsList(self.database_handler_.overdue_rooms_)

		# Merge the trashcan rooms together with the dry rooms --> Use rooms_dry_cleaning from here
		for room in rooms_trashcan_only:
			if not (room in rooms_dry_cleaning):
				rooms_dry_cleaning.append(room) 
		
		# Get the parameters for room sequencing
		if (self.is_overdue_ == False):
			if (self.is_wet_ == False):
				self.segmented_map_, self.room_information_in_pixel_ = self.database_handler_.getMapAndRoomInformationInPixel(rooms_wet_cleaning)
			else:
				self.segmented_map_, self.room_information_in_pixel_ = self.database_handler_.getMapAndRoomInformationInPixel(rooms_dry_cleaning)
		else:
			if (self.is_wet_ == False):
				self.segmented_map_, self.room_information_in_pixel_ = self.database_handler_.getMapAndRoomInformationInPixel(rooms_wet_cleaning)
			else:
				self.segmented_map_, self.room_information_in_pixel_ = self.database_handler_.getMapAndRoomInformationInPixel(rooms_dry_cleaning)

		# Sequence the rooms
		self.room_sequencer_ = room_sequencing_behavior.RoomSequencingBehavior("Room sequencing", self.interrupt_var_, self.room_sequencing_service_str_)
		self.room_sequencer_.setParameters(
			self.database_handler_,
			self.room_information_in_pixel_,
			self.robot_radius_
			)
		self.room_sequencer_.executeCustomBehavior()
		self.room_sequencing_data_ = self.room_sequencer_.room_sequence_result_	
		#self.printMsg("self.room_sequencing_data_.checkpoints=" + str(self.room_sequencing_data_.checkpoints))
		#self.printMsg("self.segmentation_data_.room_information_in_pixel=" + str(self.segmentation_data_.room_information_in_pixel))
		#self.printMsg("self.segmentation_data_.room_information_in_meter=" + str(self.segmentation_data_.room_information_in_meter))