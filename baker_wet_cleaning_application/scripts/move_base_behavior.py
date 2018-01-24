#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import *

import behavior_container

class MoveBaseBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_

	# Method for returning to the standard pose of the robot
	def setParameters(self, goal_position_, goal_orientation_, header_frame_id_):
		self.goal_position = goal_position_
		self.goal_orientation = goal_orientation_
		self.header_frame_id = header_frame_id_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current position
		# save position to go
		# if cancelled: return to previous position?
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		move_base_goal = MoveBaseGoal()
		move_base_goal.target_pose.pose.position = self.goal_position
		move_base_goal.target_pose.pose.orientation = self.goal_orientation
		move_base_goal.target_pose.header.frame_id = self.header_frame_id
		move_base_goal.target_pose.header.stamp = rospy.Time.now()
		move_base_client = actionlib.SimpleActionClient(self.service_str, MoveBaseAction)
		self.printMsg("Running move_base action...")
		self.move_base_result = self.runAction(move_base_client, move_base_goal)
		self.printMsg("move_base completed.")