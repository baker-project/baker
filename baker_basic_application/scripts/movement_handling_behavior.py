#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, Pose2D, Point32, Quaternion

import behavior_container
import move_base_behavior
import room_exploration_behavior
import move_base_path_behavior
import move_base_wall_follow_behavior

class MovementHandlingBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Serivces to be used:
	# room_exploration_service_str = 
	#       '/room_exploration/room_exploration_server'
	# move_base_path_service_str =
	#		'/move_base_path'
	# move_base_wall_follow_service_str =
	#		'/move_base_wall_follow'
	# move_base_service_str =
	#		'move_base'
	#========================================================================

		
	# Method for setting parameters for the behavior
	def setParameters(self, map_data_, sequence_data_, extraction_data_):
		# Service strings
		self.room_exploration_service_str = '/room_exploration/room_exploration_server'
		self.move_base_path_service_str = '/move_base_path'
		self.move_base_wall_follow_service_str = '/move_base_wall_follow'
		self.move_base_service_str = 'move_base'
		self.map_data = map_data_
		self.sequence_data = sequence_data_
		self.extraction_data = extraction_data_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		self.move_base_handler = move_base_behavior.MoveBaseBehavior(self.interrupt_var, self.move_base_service_str)
		self.move_base_handler.behavior_name = "Move Base handling"
		self.room_explorer = room_exploration_behavior.RoomExplorationBehavior(self.interrupt_var, self.room_exploration_service_str)
		self.room_explorer.behavior_name = "Room exploration"
		self.path_follower = move_base_path_behavior.MoveBasePathBehavior(self.interrupt_var, self.move_base_path_service_str)
		self.path_follower.behavior_name = "Path following"
		self.wall_follower = move_base_path_behavior.MoveBasePathBehavior(self.interrupt_var, self.move_base_wall_follow_service_str)
		self.wall_follower.behavior_name = "Wall following"

		for current_room_index in range(len(self.extraction_data)):

			# Robot movement into next room
			"""
			For movement to room:
			goal_position = sequence_data.checkpoints[current_room_index].checkpoint_position_in_meter
			goal_orientation = Quaternion(x=0., y=0., z=0., w=0.)
			header_frame_id = 'base_link'
			"""			
			self.move_base_handler.setParameters(
				self.sequence_data.checkpoints[current_room_index].checkpoint_position_in_meter, 
				Quaternion(x=0., y=0., z=0., w=0.), 
				'base_link'
				)
			self.move_base_handler.executeBehavior()
			
			# Interruption opportunity
			if self.handleInterrupt() == 2:
				return
			
			# Room exploration
			"""
			For room exploration:
			map_data = map_data
			input_map = extraction_data[current_room_index]
			map_resolution = self.map_data.map_resolution
			map_origin = self.map_data.map_origin
			robot_radius = 0.3
			coverage_radius = 0.3
			field_of_view = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364), Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)] # this field of view represents the off-center iMop floor wiping device
			starting_position = Pose2D(x=1., y=0., theta=0.)
			planning_mode = 2
			"""
			self.room_explorer.setParameters(
				self.map_data,
				self.extraction_data[current_room_index],
				self.map_data.map_resolution,
				self.map_data.map_origin,
				0.3,
				0.3,
				[Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364), Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)], # this field of view represents the off-center iMop floor wiping device
				Pose2D(x=1., y=0., theta=0.),
				2
			)
			self.room_explorer.executeBehavior()
			
			# Interruption opportunity
			if self.handleInterrupt() == 2:
				return

			# Explored path follow
			"""
			For path follow movement:
			target_poses = exploration_result.coverage_path_pose_stamped
			path_tolerance = 0.2
			goal_position_tolerance = 0.5
			goal_angle_tolerance = 1.57
			"""
			self.path_follower.setParameters(
				self.room_explorer.exploration_result.coverage_path_pose_stamped,
				0.2,
				0.5,
				1.57
			)
			self.path_follower.executeBehavior()
			
			# Interruption opportunity
			if self.handleInterrupt() == 2:
				return

			# Wall follow
			"""
			For wall following movement:
			target_poses = exploration_result.coverage_path_pose_stamped
			path_tolerance = 0.2
			goal_position_tolerance = 0.4
			goal_angle_tolerance = 3.14
			"""
			self.wall_follower.setParameters(
				self.room_explorer.exploration_result.coverage_path_pose_stamped,
				0.2,
				0.4,
				3.14
			)
			self.wall_follower.executeBehavior()
			
			# Interruption opportunity
			if self.handleInterrupt() == 2:
				return