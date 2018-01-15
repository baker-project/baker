#!/usr/bin/env python

import behavior_container

class MoveBasePathBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current position (pose, room)
		# return to standard pose / stop all motors
		pass

	# Method for setting parameters for the behavior
	def setParameters(self, target_poses_, path_tolerance_, goal_position_tolerance_, goal_angle_tolerance_):
		self.target_poses = target_poses_
		self.path_tolerance = path_tolerance_
		self.goal_position_tolerance = goal_position_tolerance_
		self.goal_angle_tolerance = goal_angle_tolerance_

	# Implemented Behavior
	def executeCustomBehavior(self):
		move_base_path_goal = MoveBasePathGoal()
		move_base_path_goal.target_poses = self.target_poses
		move_base_path_goal.path_tolerance = self.path_tolerance
		move_base_path_goal.goal_position_tolerance = self.goal_position_tolerance
		move_base_path_goal.goal_angle_tolerance = self.goal_angle_tolerance
		print "Waiting for action '" + str(self.service_str) + "' to become available ..."
		move_base_path_client = actionlib.SimpleActionClient(self.service_str, MoveBasePathAction)
		move_base_path_result = self.runAction(move_base_path_client, move_base_path_goal)