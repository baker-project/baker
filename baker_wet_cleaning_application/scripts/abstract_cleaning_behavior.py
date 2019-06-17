#!/usr/bin/env python

from move_base_behavior import MoveBaseBehavior
from room_exploration_behavior import RoomExplorationBehavior
from tool_changing_behavior import ToolChangingBehavior
from trolley_movement_behavior import TrolleyMovementBehavior
from behavior_container import BehaviorContainer

from utils import getCurrentRobotPosition
from geometry_msgs.msg import Pose2D, Quaternion
from copy import copy
import services_params as srv


class AbstractCleaningBehavior(BehaviorContainer):

	#========================================================================
	# Description:
	# Hangles a cleaning process (can be a dry or a wet cleaning_behavior)
	# for all rooms provided in a given list
	#========================================================================

	def __init__(self, behavior_name, interrupt_var):
		super(AbstractCleaningBehavior, self).__init__(behavior_name, interrupt_var)
		(self.move_base_handler_, self.tool_changer_, self.trolley_mover_) = (None, None, None)

	# Method for returning to the standard state of the robot
	def returnToRobotStandardState(self):
		# nothing to be saved
		# nothing to be undone
		pass

	def computeCoveragePath(self, room_id):
		self.printMsg('Starting computing coverage path of room ID {}'.format(room_id))

		room_explorer = RoomExplorationBehavior("RoomExplorationBehavior",
																		  self.interrupt_var_,
																		  self.room_exploration_service_str_)

		room_center = self.room_information_in_meter_[room_id].room_center
		room_map_data = self.database_handler_.database_.getRoomById(room_id).room_map_data_
		(robot_position, _, _) = getCurrentRobotPosition()
		starting_position = (robot_position[0], robot_position[1]) if robot_position is not None else (room_center.x, room_center.y)
		room_explorer.setParameters(
			input_map=room_map_data,
			map_resolution=self.database_handler_.database_.global_map_data_.map_resolution_,
			map_origin=self.database_handler_.database_.global_map_data_.map_origin_,
			robot_radius=self.robot_radius_,
			coverage_radius=self.coverage_radius_,
			field_of_view=self.field_of_view_,  # this field of view represents the off-center iMop floor wiping device
			field_of_view_origin=self.field_of_view_origin_,
			starting_position=Pose2D(x=starting_position[0], y=starting_position[1], theta=0.),
			# todo: determine theta
			planning_mode=2
		)
		room_explorer.executeBehavior()
		self.printMsg('Coverage path of room ID {} computed.'.format(room_id))

		return room_explorer.exploration_result_.coverage_path_pose_stamped

	# todo (rmb-ma). dry or wet ??
	def checkoutRoom(self, room_id, nb_found_dirtspots=0, nb_found_trashcans=0):
		self.printMsg("checkout dry cleaned room: " + str(room_id))

		cleaning_tasks = copy(self.database_handler_.database_.getRoomById(room_id).open_cleaning_tasks_)
		for task in cleaning_tasks:
			self.database_handler_.checkoutCompletedRoom(
				self.database_handler_.database_.getRoomById(room_id),
				assignment_type=task)

		# Adding log entry for dry cleaning (but two  )
		self.database_handler_.addLogEntry(
			room_id=room_id,
			status=1,  # 1=Completed
			cleaning_task=0,  # 1=wet only
			found_dirtspots=nb_found_dirtspots,
			found_trashcans=nb_found_trashcans,
			cleaned_surface_area=0,
			room_issues=[],
			used_water_amount=0,
			battery_usage=0
		)

	def getCheckpointForRoomId(self, room_id):
		room_counter = 0
		for checkpoint in self.sequencing_result_.checkpoints:
			for _ in checkpoint.room_indices:
				current_room_id = self.mapping_[room_counter]
				if current_room_id == room_id:
					return checkpoint
				room_counter += 1
		assert False

	def executeCustomBehaviorInRoomId(self, room_id):
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		self.move_base_handler_ = MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_,	srv.MOVE_BASE_SERVICE_STR)
		self.tool_changer_ = ToolChangingBehavior("ToolChangingBehavior", self.interrupt_var_)
		self.trolley_mover_ = TrolleyMovementBehavior("TrolleyMovingBehavior", self.interrupt_var_)
		# Tool change according to cleaning task
		self.tool_changer_.setParameters(self.database_handler_)
		self.tool_changer_.executeBehavior()

		room_counter = 0
		for checkpoint in self.sequencing_result_.checkpoints:
			# Trolley movement to checkpoint
			self.trolley_mover_.setParameters(self.database_handler_)
			self.trolley_mover_.executeBehavior()

			self.move_base_handler_.setParameters(
				goal_position=checkpoint.checkpoint_position_in_meter,
				goal_orientation=Quaternion(x=0., y=0., z=0., w=1.),
				header_frame_id='base_link'
			)
			self.move_base_handler_.executeBehavior()

			for _ in checkpoint.room_indices:
				current_room_id = self.mapping_[room_counter]
				self.executeCustomBehaviorInRoomId(room_id=current_room_id)
				room_counter += 1
