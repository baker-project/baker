#!/usr/bin/env python

import roslib
import actionlib
import rospy
from std_msgs.msg import String
from ipa_building_msgs.msg import *

import behavior_container

class MapSegmentationBehavior(behavior_container.BehaviorContainer):

	def __init__(self, behavior_name, interrupt_var, service_str):
		self.behavior_name_ = behavior_name
		self.interrupt_var_ = interrupt_var
		self.service_str_ = service_str

	# Method for setting parameters for the behavior
	def setParameters(self, map_data):
		self.map_data_ = map_data

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
		# rospy.init_node('exploration_node') idk why this is here...
		segmentation_goal.map_resolution = self.map_data_.map_resolution
		segmentation_goal.map_origin = self.map_data_.map_origin
		segmentation_goal.return_format_in_meter = True
		segmentation_goal.return_format_in_pixel = True
		segmentation_goal.robot_radius = 0.3	# todo: receive as parameter
		segmentation_client = actionlib.SimpleActionClient(str(self.service_str_), MapSegmentationAction)
		self.printMsg("Running segmentation action...")
		self.segmentation_result_ = self.runAction(segmentation_client, segmentation_goal)
		self.printMsg("Map Segmentation completed")
