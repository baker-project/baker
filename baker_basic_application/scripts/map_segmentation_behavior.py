#!/usr/bin/env python

import roslib
import actionlib
import rospy
from std_msgs.msg import String
from ipa_building_msgs.msg import *

import behavior_container

class MapSegmentationBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_, map_data_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_
		self.map_data = map_data_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no data to save
		# nothing to be undone
		pass
	
	# Implemented Behavior
	def executeCustomBehavior(self):
		# compute map division into rooms (ipa_room_segmentation)
		segmentation_goal = MapSegmentationGoal()
		segmentation_goal.input_map = self.map_data.map
		# rospy.init_node('exploration_node') idk why this is here...
		segmentation_goal.map_resolution = self.map_data.map_resolution
		segmentation_goal.map_origin = self.map_data.map_origin
		segmentation_goal.return_format_in_meter = False
		segmentation_goal.return_format_in_pixel = True
		segmentation_goal.robot_radius = 0.3
		print "Waiting for Action " + str(self.service_str) + " to become available..."
		segmentation_client = actionlib.SimpleActionClient(str(self.service_str), MapSegmentationAction)
		print "Running segmentation action..."
		self.segmentation_result = self.runAction(segmentation_client, segmentation_goal)
		print "Map Segmentation completed"