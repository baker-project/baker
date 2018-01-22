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
	# map_receiving_service_str:
	#       '/baker/get_map_image'
	# map_segmentation_service_str:
	#       '/room_segmentation/room_segmentation_server'
	# room_sequencing_service_str = 
	#       '/room_sequence_planning/room_sequence_planning_server'
	#========================================================================
	
	# Method for returning to the standard pose of the robot
	def setParameters(self):
		self.map_receiving_service_str = '/baker/get_map_image'
		self.map_segmentation_service_str = '/room_segmentation/room_segmentation_server'
		self.room_sequencing_service_str = '/room_sequence_planning/room_sequence_planning_server'

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# data to be saved
		# ...
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):

		# Map receiving
		self.map_receiver = map_receiving_behavior.MapReceivingBehavior(self.interrupt_var, self.map_receiving_service_str) 
		self.map_receiver.behavior_name = "Map receiving"
		self.map_receiver.setParameters()
		self.map_receiver.executeCustomBehavior()
		self.map_data = self.map_receiver.map_data

		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		
		# Map segmentation
		self.map_segmenter = map_segmentation_behavior.MapSegmentationBehavior(self.interrupt_var, self.map_segmentation_service_str)
		self.map_segmenter.behavior_name = "Map segmentation"
		self.map_segmenter.setParameters(
			self.map_data
			)
		self.map_segmenter.executeCustomBehavior()
		self.segmentation_data = self.map_segmenter.segmentation_result
		
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		
		# Room sequencing
		self.room_sequencer = room_sequencing_behavior.RoomSequencingBehavior(self.interrupt_var, self.room_sequencing_service_str)
		self.room_sequencer.behavior_name = "Room sequencing"
		self.room_sequencer.setParameters(
			self.map_data, 
			self.segmentation_data
			)
		self.room_sequencer.executeCustomBehavior()
		self.room_sequencing_data = self.room_sequencer.room_sequence_result
		
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
			
		self.printMsg("self.room_sequencing_data.checkpoints=")
		print self.room_sequencing_data.checkpoints
		self.printMsg("self.segmentation_data.room_information_in_pixel=")
		print self.segmentation_data.room_information_in_pixel
		self.printMsg("self.segmentation_data.room_information_in_meter=")
		print self.segmentation_data.room_information_in_meter
		
		"""
		# Room map extraction and conversion
		self.room_extractor = map_segment_extracting_behavior.MapSegmentExtractingBehavior(self.interrupt_var)
		self.room_extractor.behavior_name = "Room extraction"
		self.room_extractor.setParameters(
			self.segmentation_data, 
			self.room_sequencing_data
			)
		self.room_extractor.executeCustomBehavior()
		self.room_extraction_data = self.room_extractor.extraction_result
		"""