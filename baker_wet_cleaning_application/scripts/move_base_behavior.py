#!/usr/bin/env python

import actionlib
import rospy
from scitos_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped
from utils import projectToFrame
from math import pi

from behavior_container import BehaviorContainer

class MoveBaseBehavior(BehaviorContainer):

	# ========================================================================
	# Description:
	# Class which contains the behavior for moving the robot to
	# a specified position
	# ========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		super(MoveBaseBehavior, self).__init__(behavior_name, interrupt_var)
		self.service_str_ = service_str

		(self.goal_position_, self.goal_orientation_) = (None, None)
		self.header_frame_id_ = None
		(self.goal_position_tolerance_, self.goal_angle_tolerance_) = (None, None)

	# Method for returning to the standard pose of the robot
	def setParameters(self, goal_position, goal_orientation, header_frame_id='map',
					  goal_position_tolerance=0.2, goal_angle_tolerance=0.2, time=None):

		assert(header_frame_id == 'map')

		self.goal_position_ = goal_position
		self.goal_orientation_ = goal_orientation
		self.header_frame_id_ = header_frame_id
		self.goal_position_tolerance_ = goal_position_tolerance
		self.goal_angle_tolerance_ = goal_angle_tolerance

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current position
		# save position to go
		# if cancelled: return to previous position?
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		assert self.goal_position_ is not None

		move_base_goal = MoveBaseGoal()
		move_base_goal.target_pose.pose.position = self.goal_position_
		move_base_goal.target_pose.pose.orientation = self.goal_orientation_
		move_base_goal.target_pose.header.frame_id = self.header_frame_id_
		move_base_goal.target_pose.header.stamp = rospy.Time.now()
		move_base_goal.goal_position_tolerance = self.goal_position_tolerance_
		move_base_goal.goal_angle_tolerance = self.goal_angle_tolerance_
		move_base_client = actionlib.SimpleActionClient(self.service_str_, MoveBaseAction)
		self.printMsg("Running move_base action...")
		self.move_base_result_ = self.runAction(move_base_client, move_base_goal)['result']
		self.printMsg("move_base completed.")
