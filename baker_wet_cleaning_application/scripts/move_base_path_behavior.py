#!/usr/bin/env python

import actionlib

from scitos_msgs.msg import MoveBasePathAction, MoveBasePathGoal

import behavior_container

class MoveBasePathBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Class which contains the behavior for making the robot follow 
	# a specified trajectory
	#========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		self.behavior_name_ = behavior_name
		self.interrupt_var_ = interrupt_var
		self.service_str_ = service_str

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current position (pose, room)
		# return to standard pose / stop all motors
		pass

	# Method for setting parameters for the behavior
	def setParameters(self, target_poses, area_map, path_tolerance, goal_position_tolerance, goal_angle_tolerance):
		self.target_poses_ = target_poses
		self.area_map_ = area_map
		self.path_tolerance_ = path_tolerance
		self.goal_position_tolerance_ = goal_position_tolerance
		self.goal_angle_tolerance_ = goal_angle_tolerance

	# Implemented Behavior
	def executeCustomBehavior(self):
		move_base_path_goal = MoveBasePathGoal()
		move_base_path_goal.target_poses = self.target_poses_
		move_base_path_goal.area_map = self.area_map_
		move_base_path_goal.path_tolerance = self.path_tolerance_
		move_base_path_goal.goal_position_tolerance = self.goal_position_tolerance_
		move_base_path_goal.goal_angle_tolerance = self.goal_angle_tolerance_
		move_base_path_client = actionlib.SimpleActionClient(self.service_str_, MoveBasePathAction)
		self.printMsg("Running move_base_path action...")
		self.move_base_path_result_ = self.runAction(move_base_path_client, move_base_path_goal)
		self.printMsg("move_base_path completed.")