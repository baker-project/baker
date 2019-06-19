#!/usr/bin/env python

import actionlib
from scitos_msgs.msg import MoveBaseWallFollowAction
from scitos_msgs.msg import MoveBaseWallFollowGoal
import behavior_container


class MoveBaseWallFollowBehavior(behavior_container.BehaviorContainer):

	#  ========================================================================
	# Description:
	# Class which contains the behavior for making the robot follow along
	# the walls
	#  ========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		super(MoveBaseWallFollowBehavior, self).__init__(behavior_name, interrupt_var)
		self.service_str_ = service_str

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current position (pose, room)
		# return to standard pose / stop all motors
		pass

	# Method for setting parameters for the behavior
	def setParameters(self, map, area_map, coverage_map, map_resolution, map_origin, path_tolerance,
					  goal_position_tolerance, goal_angle_tolerance, target_wall_distance, wall_following_off_traveling_distance_threshold):
		self.map_ = map		# contains map, map_resolution, map_origin
		self.area_map_ = area_map
		self.coverage_map_ = coverage_map
		self.map_resolution_ = map_resolution
		self.map_origin_ = map_origin
		self.path_tolerance_ = path_tolerance
		self.goal_position_tolerance_ = goal_position_tolerance
		self.goal_angle_tolerance_ = goal_angle_tolerance
		self.target_wall_distance_ = target_wall_distance
		self.wall_following_off_traveling_distance_threshold_ = wall_following_off_traveling_distance_threshold

	# Implemented Behavior
	def executeCustomBehavior(self):
		move_base_goal = MoveBaseWallFollowGoal()
		move_base_goal.map = self.map_
		move_base_goal.area_map = self.area_map_
		move_base_goal.coverage_map = self.coverage_map_
		move_base_goal.map_resolution = self.map_resolution_
		move_base_goal.map_origin = self.map_origin_
		move_base_goal.path_tolerance = self.path_tolerance_
		move_base_goal.goal_position_tolerance = self.goal_position_tolerance_
		move_base_goal.goal_angle_tolerance = self.goal_angle_tolerance_
		move_base_goal.target_wall_distance = self.target_wall_distance_
		move_base_goal.wall_following_off_traveling_distance_threshold = self.wall_following_off_traveling_distance_threshold_
		move_base_client = actionlib.SimpleActionClient(self.service_str_, MoveBaseWallFollowAction)
		self.printMsg("Running move_base_wall_follow action...")
		self.move_base_wall_follow_result_ = self.runAction(move_base_client, move_base_goal)['result']
		self.printMsg("move_base_wall_follow completed.")
