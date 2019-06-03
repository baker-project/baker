#!/usr/bin/env python

import actionlib
import rospy
from geometry_msgs.msg import Point32, Quaternion
from ipa_building_msgs.msg import FindRoomSequenceWithCheckpointsAction, FindRoomSequenceWithCheckpointsGoal
import behavior_container
from utils import getCurrentRobotPosition

class RoomSequencingBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Class which contains the behavior for sequencing a list of rooms
	#========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		super(RoomSequencingBehavior, self).__init__(behavior_name, interrupt_var)
		self.service_str_ = service_str

	# Method for setting parameters for the behavior
	#def setParameters(self, map_data, segmentation_data, robot_radius):
	def setParameters(self, database, room_information_in_pixel, robot_radius):
		self.database_ = database
		self.room_information_in_pixel_ = room_information_in_pixel
		self.robot_radius_ = robot_radius

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no data to save
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):

		room_sequence_goal = FindRoomSequenceWithCheckpointsGoal()
		room_sequence_goal.input_map = self.database_.global_map_data_.map_image_
		room_sequence_goal.map_resolution = self.database_.global_map_data_.map_resolution_
		room_sequence_goal.map_origin = self.database_.global_map_data_.map_origin_
		room_sequence_goal.robot_radius = self.robot_radius_
		room_sequence_goal.room_information_in_pixel = self.room_information_in_pixel_
		(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = getCurrentRobotPosition()
		if robot_pose_translation is not None:
			room_sequence_goal.robot_start_coordinate.position = Point32(x=robot_pose_translation[0], y=robot_pose_translation[1])  # actual current coordinates should be inserted
		else:
			self.printMsg("Warning: tf lookup failed, taking (0,0) as robot_start_coordinate.")
			room_sequence_goal.robot_start_coordinate.position = Point32(x=0, y=0)
		room_sequence_goal.robot_start_coordinate.orientation = Quaternion(x=0., y=0., z=0., w=0.)  # todo: normalized quaternion
		room_sequence_client = actionlib.SimpleActionClient(str(self.service_str_), FindRoomSequenceWithCheckpointsAction)
		self.printMsg("Running sequencing action...")
		self.room_sequence_result_ = self.runAction(room_sequence_client, room_sequence_goal)['result']
		self.printMsg("Room sequencing completed.")