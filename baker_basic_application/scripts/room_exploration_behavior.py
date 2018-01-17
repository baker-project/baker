#!/usr/bin/env python

import rospy
import actionlib

from ipa_building_msgs.msg import *

import behavior_container

class RoomExplorationBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Method for setting parameters for the behavior
	def setParameters(self, map_data_, input_map_, map_resolution_, map_origin_, robot_radius_, coverage_radius_, field_of_view_, starting_position_, planning_mode_):
		self.map_data = map_data_
		self.input_map = input_map_
		self.map_resolution = map_resolution_
		self.map_origin = map_origin_
		self.robot_radius = robot_radius_
		self.coverage_radius = coverage_radius_
		self.field_of_view = field_of_view_
		self.starting_position = starting_position_
		self.planning_mode = planning_mode_

	# Implemented Behavior
	def executeCustomBehavior(self):
		exploration_goal = RoomExplorationGoal()
		exploration_goal.input_map = self.input_map
		exploration_goal.map_resolution = self.map_resolution
		exploration_goal.map_origin = self.map_origin
		exploration_goal.robot_radius = self.robot_radius
		exploration_goal.coverage_radius = self.coverage_radius
		exploration_goal.field_of_view = self.field_of_view
		exploration_goal.starting_position = self.starting_position
		exploration_goal.planning_mode = self.planning_mode
		exploration_client = actionlib.SimpleActionClient(self.service_str, RoomExplorationAction)
		self.printMsg("Running room exploration action...")
		self.exploration_result = self.runAction(exploration_client, exploration_goal)
		self.printMsg("Exploration path received with length " + str(len(self.exploration_result.coverage_path_pose_stamped)))
		self.printMsg("Room exploration action completed.")