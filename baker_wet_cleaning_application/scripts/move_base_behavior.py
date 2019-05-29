#!/usr/bin/env python

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

from scripts.tests import behavior_container


class MoveBaseBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Class which contains the behavior for moving the robot to 
	# a specified position
	#========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		super(MoveBaseBehavior, self).__init__(behavior_name, interrupt_var)
		self.service_str_ = service_str

	# Method for returning to the standard pose of the robot
	def setParameters(self, goal_position, goal_orientation, header_frame_id):
		self.goal_position_ = goal_position
		self.goal_orientation_ = goal_orientation
		self.header_frame_id_ = header_frame_id

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current position
		# save position to go
		# if cancelled: return to previous position?
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		move_base_goal = MoveBaseGoal()
		move_base_goal.target_pose.pose.position = self.goal_position_
		move_base_goal.target_pose.pose.orientation = self.goal_orientation_
		move_base_goal.target_pose.header.frame_id = self.header_frame_id_
		move_base_goal.target_pose.header.stamp = rospy.Time.now()
		move_base_client = actionlib.SimpleActionClient(self.service_str_, MoveBaseAction)
		self.printMsg("Running move_base action...")
		self.move_base_result_ = self.runAction(move_base_client, move_base_goal)['result']
		self.printMsg("move_base completed.")