#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import behavior_container
import map_receiving_behavior
import map_segmentation_behavior
import room_sequencing_behavior
import map_segment_extracting_behavior

class MapHandlingBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, map_receiving_service_str_, map_segmentation_service_str_, room_sequencing_service_str_):
		super(MapHandlingBehavior, self).__init__(interrupt_var_)
		self.map_receiving_service_str = map_receiving_service_str_
		self.map_segmentation_service_str = map_segmentation_service_str_
		self.room_sequencing_service_str = room_sequencing_service_str_

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
		self.map_receiver.executeCustomBehavior()
		self.map_data = self.map_receiver.map_data
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		# Map segmentation
		self.map_segmenter = map_segmentation_behavior.MapSegmentationBehavior(self.interrupt_var, self.map_segmentation_service_str, self.map_data)
		self.map_segmenter.executeCustomBehavior()
		self.segmentation_data = self.map_segmenter.segmentation_result
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		# Room sequencing
		self.room_sequencer = room_sequencing_behavior.RoomSequencingBehavior(self.interrupt_var, self.room_sequencing_service_str, self.map_data, self.segmentation_data)
		self.room_sequencer.executeCustomBehavior()
		self.room_sequencing_data = self.room_sequencer.room_sequence_result
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		# Room map extraction and conversion
		self.room_extractor = map_segment_extracting_behavior.MapSegmentExtractingBehavior(self.interrupt_var, self.segmentation_data, self.room_sequencing_data)
		self.room_extractor.executeCustomBehavior()
		self.room_extraction_data = self.room_extractor.extracted_maps