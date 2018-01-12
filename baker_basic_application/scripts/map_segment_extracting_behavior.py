#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import behavior_container

class MapSegmentExtractingBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, segmentation_data_, room_sequence_data_):
		self.interrupt_var = interrupt_var_
		self.segmentation_data = segmentation_data_
		self.room_sequence_data = room_sequence_data_
		# Get a opencv representation of the segmented image
		self.bridge = CvBridge()
		self.opencv_segmented_map = self.bridge.imgmsg_to_cv2(self.segmentation_data.segmented_map, desired_encoding = "passthrough")

	# Method for returning the segment of the map corresponding to the order number as cv_bridge
	def getMapSegmentAsCVBridge(self, room_sequence_index_):
		print "Creating room map for room %i" % room_sequence_index_
		image_height, image_width = self.opencv_segmented_map.shape
		tmp_map_opencv = np.zeros((image_width, image_height), np.uint8)
		current_room_index = self.room_sequence_data.checkpoints[room_sequence_index_].room_indices[0]
		for x in range(image_width):
			for y in range(image_height):
				if (self.opencv_segmented_map[y, x] == current_room_index + 1):
					tmp_map_opencv[y, x] = 255
					print "%i %i %i" % (self.opencv_segmented_map[y, x], x, y)			
		return self.bridge.cv2_to_imgmsg(tmp_map_opencv, encoding = "mono8")

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		self.extraction_result = []
		print "Rooms to extract: %i" % len(self.room_sequence_data.checkpoints)
		for current_room in range(0, len(self.room_sequence_data.checkpoints) - 1):
			self.extraction_result.append(self.getMapSegmentAsCVBridge(current_room))