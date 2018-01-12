#!/usr/bin/env python


import behavior_container

class MoveBasePathBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_, map_data_, input_map_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_
		self.map_data = map_data_
		self.input_map = input_map_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		# After each command, def handleInterrupt has to be executed:
		# if self.handleInterrupt() == 2:
		#     return
		# explore the current room
		planning_mode = 2 # viewpoint planning
		fov_points = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364), Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)] # this field of view represents the off-center iMop floor wiping device
		exploration_goal = RoomExplorationGoal()
		exploration_goal.input_map = tmp_map_bridge
		exploration_goal.map_resolution = self.map_data.map_resolution
		exploration_goal.map_origin = self.map_data.map_origin
		exploration_goal.robot_radius = 0.3
		exploration_goal.coverage_radius = 0.3
		exploration_goal.field_of_view = fov_points
		exploration_goal.starting_position = Pose2D(x=1., y=0., theta=0.)
		exploration_goal.planning_mode = planning_mode
		print "Waiting for action '" + self.service_str + ""' to become available ..."
		exploration_client = actionlib.SimpleActionClient(self.service_str, RoomExplorationAction)
		exploration_result = self.runAction(exploration_client, exploration_goal)
		print "Exploration path received with length ", len(exploration_result.coverage_path_pose_stamped)