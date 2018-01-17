#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose2D, Point32, Quaternion
from ipa_building_msgs.msg import *

import behavior_container

class RoomSequencingBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_

	# Method for setting parameters for the behavior
	def setParameters(self, map_data_, segmentation_data_):
		self.map_data = map_data_
		self.segmentation_data = segmentation_data_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no data to save
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		room_sequence_goal = FindRoomSequenceWithCheckpointsGoal()
		room_sequence_goal.input_map = self.map_data.map
		room_sequence_goal.map_resolution = self.map_data.map_resolution
		room_sequence_goal.map_origin = self.map_data.map_origin
		room_sequence_goal.robot_radius = 0.3
		room_sequence_goal.room_information_in_pixel = self.segmentation_data.room_information_in_pixel
		room_sequence_goal.robot_start_coordinate.position = Point32(x=8., y=14.)  # actual current coordinates should be inserted
		room_sequence_goal.robot_start_coordinate.orientation = Quaternion(x=0.,y=0.,z=0., w=0.)
		room_sequence_client = actionlib.SimpleActionClient(str(self.service_str), FindRoomSequenceWithCheckpointsAction)
		self.printMsg("Running sequencing action...")
		self.room_sequence_result = self.runAction(room_sequence_client, room_sequence_goal)
		self.printMsg("Room sequencing completed.")