#!/usr/bin/env python

import actionlib
from ipa_building_msgs.msg import MapSegmentationGoal, mapSegmentationAction

import behavior_container


class MapSegmentationBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Class which contains the behavior for segmenting a map
	#========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		super(MapSegmentationBehavior, self).__init__(behavior_name, interrupt_var)
		self.service_str_ = service_str

	# Method for setting parameters for the behavior
	def setParameters(self, map_data, robot_radius, map_segmentation_algorithm=0):
		self.map_data_ = map_data
		self.robot_radius_ = robot_radius
		self.map_segmentation_algorithm_ = map_segmentation_algorithm

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no data to save
		# nothing to be undone
		pass
	
	# Implemented Behavior
	def executeCustomBehavior(self):
		# compute map division into rooms (ipa_room_segmentation)
		segmentation_goal = MapSegmentationGoal()
		segmentation_goal.input_map = self.map_data_.map
		segmentation_goal.map_resolution = self.map_data_.map_resolution
		segmentation_goal.map_origin = self.map_data_.map_origin
		segmentation_goal.return_format_in_meter = True
		segmentation_goal.return_format_in_pixel = True
		segmentation_goal.robot_radius = self.robot_radius_
		segmentation_goal.room_segmentation_algorithm = self.map_segmentation_algorithm_
		segmentation_client = actionlib.SimpleActionClient(str(self.service_str_), MapSegmentationAction)
		self.printMsg("Running segmentation action...")
		self.segmentation_result_ = self.runAction(segmentation_client, segmentation_goal)['result']
		self.printMsg("Map Segmentation completed")
