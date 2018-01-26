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
	def setParameters(self):
		self.map_receiving_service_str_ = '/map_management_client/get_map_image'
		self.map_segmentation_service_str_ = '/room_segmentation/room_segmentation_server'
		self.room_sequencing_service_str_ = '/room_sequence_planning/room_sequence_planning_server'

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# data to be saved
		# ...
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):

		# Map receiving
		self.map_receiver_ = map_receiving_behavior.MapReceivingBehavior("Map receiving", self.interrupt_var_, self.map_receiving_service_str_)
		self.map_receiver_.setParameters()
		self.map_receiver_.executeCustomBehavior()
		self.map_data_ = self.map_receiver_.map_data_

		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		
		# Map segmentation
		self.map_segmenter_ = map_segmentation_behavior.MapSegmentationBehavior("Map segmentation", self.interrupt_var_, self.map_segmentation_service_str_)
		self.map_segmenter_.setParameters(
			self.map_data_
			)
		self.map_segmenter_.executeCustomBehavior()
		self.segmentation_data_ = self.map_segmenter_.segmentation_result_
		
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		
		# Room sequencing
		self.room_sequencer_ = room_sequencing_behavior.RoomSequencingBehavior("Room sequencing", self.interrupt_var_, self.room_sequencing_service_str_)
		self.room_sequencer_.setParameters(
			self.map_data_, 
			self.segmentation_data_
			)
		self.room_sequencer_.executeCustomBehavior()
		self.room_sequencing_data_ = self.room_sequencer_.room_sequence_result_
		
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
			
		self.printMsg("self.room_sequencing_data_.checkpoints=")
		print self.room_sequencing_data_.checkpoints
		self.printMsg("self.segmentation_data_.room_information_in_pixel=")
		print self.segmentation_data_.room_information_in_pixel
		self.printMsg("self.segmentation_data_.room_information_in_meter=")
		print self.segmentation_data_.room_information_in_meter