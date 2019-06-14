#!/usr/bin/env python

from threading import Lock, Thread
from copy import copy

import rospy
from cob_object_detection_msgs.msg import DetectionArray
from geometry_msgs.msg import Pose2D, Quaternion
from std_srvs.srv import Empty

import move_base_behavior
import move_base_path_behavior
import room_exploration_behavior
import services_params as srv
import tool_changing_behavior
import trolley_movement_behavior
from dirt_removing_behavior import DirtRemovingBehavior
import behavior_container
from trashcan_emptying_behavior import TrashcanEmptyingBehavior
from utils import getCurrentRobotPosition


class DryCleaningBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Handles the dry cleaning process (i.e. Exploring, Dirt removal, Trashcan)
	# for all rooms provided in a given list
	#========================================================================
	@staticmethod
	def containsTrashcanTask(tasks):
		return -1 in tasks

	@staticmethod
	def containsDirtTask(tasks):
		return 0 in tasks

	def __init__(self, behavior_name, interrupt_var):
		super(DryCleaningBehavior, self).__init__(behavior_name, interrupt_var)
		(self.detected_dirts_, self.detected_trashs_) = ([], [])
		self.local_mutex_ = Lock()
		(self.trash_topic_subscriber_, self.dirt_topic_subscriber_) = (None, None)
		(self.found_dirtspots_, self.found_trashcans_) = ([], [])
		(self.move_base_handler_, self.tool_changer_, self.trolley_mover_) = (None, None, None)


	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, sequencing_result, mapping, robot_radius, coverage_radius, field_of_view,
					  field_of_view_origin, room_information_in_meter):
		self.database_handler_ = database_handler
		self.sequencing_result_ = sequencing_result
		self.mapping_ = mapping
		self.room_exploration_service_str_ = srv.ROOM_EXPLORATION_SERVICE_STR
		self.move_base_path_service_str_ = srv.MOVE_BASE_PATH_SERVICE_STR

		self.robot_radius_ = robot_radius
		self.coverage_radius_ = coverage_radius
		self.field_of_view_ = field_of_view  # this field of view represents the off-center iMop floor wiping device
		self.field_of_view_origin_ = field_of_view_origin
		self.room_information_in_meter_ = room_information_in_meter

	# Method for returning to the standard state of the robot
	def returnToRobotStandardState(self):
		# nothing to be saved
		# nothing to be undone
		pass

	# Empty the trash detected in self.detected_trash_
	def trashcanRoutine(self, room_id, detected_trash):
		assert(detected_trash is not None)

		self.found_trashcans_.append(detected_trash)
		trashcan_emptier = TrashcanEmptyingBehavior("TrashcanEmptyingBehavior", self.interrupt_var_, srv.MOVE_BASE_SERVICE_STR)

		position = detected_trash.pose.pose.position
		checkpoint_position = self.getCheckpointForRoomId(room_id).checkpoint_position_in_meter

		trashcan_emptier.setParameters(trashcan_position=position, trolley_position=checkpoint_position)

		trashcan_emptier.executeBehavior()

	# Remove the dirt detected in self.detected_dirt_
	def dirtRoutine(self, room_id, detected_dirt):
		assert(detected_dirt is not None)

		self.found_dirtspots_.append(detected_dirt)
		dirt_remover = DirtRemovingBehavior("DirtRemovingBehavior", self.interrupt_var_,
											move_base_service_str=srv.MOVE_BASE_SERVICE_STR,
											map_accessibility_service_str=srv.MAP_ACCESSIBILITY_SERVICE_STR)

		position = detected_dirt.pose.pose.position
		dirt_remover.setParameters(dirt_position=position)

		dirt_remover.executeBehavior()

	def callEmptyService(self, service):
		rospy.wait_for_service(service)
		try:
			rospy.ServiceProxy(service, Empty)()
			self.printMsg('Called ' + service)
		except rospy.ServiceException, e:
			self.printMsg("Service call failed: %s" % e)

	def stopDetections(self):
		Thread(target=self.callEmptyService, args=(srv.STOP_DIRT_DETECTOR_SERVICE_STR,)).start()
		Thread(target=self.callEmptyService, args=(srv.STOP_TRASH_DETECTOR_SERVICE_STR,)).start()

	@staticmethod
	def isAlreadyDetected(detections, new_detection):
		new_position = new_detection.pose.pose.position
		for detection in detections:
			position = detection.pose.pose.position
			if position.x == new_position.x and position.y == new_position.y:
				print("Detection on pos ({}, {}) is already detected".format(new_position.x, new_position.y))
				return True
		return False

	def dirtDetectionCallback(self, detections):
		detections = detections.detections
		detections = list(filter(lambda detection: not self.isAlreadyDetected(self.found_dirtspots_, detection), detections))
		if len(detections) == 0:
			return

		self.printMsg("DIRT(S) DETECTED!!")

		# 1. Stop the dirt and the trash detections
		self.stopDetections()

		# 2. Stop the path follower
		self.local_mutex_.acquire()
		self.detected_dirts_ = detections
		self.local_mutex_.release()

		position = self.detected_dirts_[0].pose.pose.position
		print("FIRST ON POSITION ({}, {})".format(position.x, position.y))

	def trashDetectionCallback(self, detections):
		detections = detections.detections
		detections = list(filter(lambda detection: not self.isAlreadyDetected(self.found_trashcans_, detection), detections))
		if len(detections) == 0:
			return
		self.printMsg("Trash(S) DETECTED!!")

		# 1. Stop the dirt and the trash detections
		self.stopDetections()

		# 2. Stop the path follower
		self.local_mutex_.acquire()
		self.detected_trashs_ = detections
		self.local_mutex_.release()

		position = self.detected_trashs_[0].pose.pose.position
		print("FIRST ON POSITION ({}, {})".format(position.x, position.y))

	def computeCoveragePath(self, room_id):
		self.printMsg('Starting computing coverage path of room ID {}'.format(room_id))

		room_explorer = room_exploration_behavior.RoomExplorationBehavior("RoomExplorationBehavior",
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

	def checkoutRoom(self, room_id):
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
			found_dirtspots=len(self.found_dirtspots_),
			found_trashcans=len(self.found_trashcans_),
			cleaned_surface_area=0,
			room_issues=[],
			used_water_amount=0,
			battery_usage=0
		)

	def executeCustomBehaviorInRoomId(self, room_id):
		cleaning_tasks = self.database_handler_.database_.getRoomById(room_id).open_cleaning_tasks_
		assert(DryCleaningBehavior.containsTrashcanTask(cleaning_tasks) or DryCleaningBehavior.containsDirtTask(cleaning_tasks))

		self.printMsg('Starting Dry Cleaning of room ID {}'.format(room_id))

		starting_position = self.room_information_in_meter_[room_id].room_center
		self.move_base_handler_.setParameters(
			goal_position=starting_position,
			goal_orientation=Quaternion(x=0., y=0., z=0., w=1.),
			header_frame_id='base_link'
		)

		thread_move_to_the_room = Thread(target=self.move_base_handler_.executeBehavior)
		thread_move_to_the_room.start()

		path = self.computeCoveragePath(room_id=room_id)

		if self.handleInterrupt() >= 1:
			return

		path_follower = move_base_path_behavior.MoveBasePathBehavior("MoveBasePathBehavior_PathFollowing",
																			self.interrupt_var_,
																			self.move_base_path_service_str_)

		if DryCleaningBehavior.containsTrashcanTask(cleaning_tasks):
			self.trash_topic_subscriber_ = rospy.Subscriber('trash_detector_topic', DetectionArray,
															self.trashDetectionCallback)

		if DryCleaningBehavior.containsDirtTask(cleaning_tasks):
			self.dirt_topic_subscriber_ = rospy.Subscriber('dirt_detector_topic', DetectionArray,
														   self.dirtDetectionCallback)

		# todo (rmb-ma). Hack to display computed path
		#with open('/home/rmb/Desktop/rmb-ma_notes/path_visualizer/path.txt', 'w') as f:
		#	f.write(str(path))

		thread_move_to_the_room.join()  # don't start the detections before
		if self.move_base_handler_.failed():
			self.printMsg('Room center is not accessible. Failed to clean room {}'.format(room_id))
			return

		while len(path) > 0:
			(self.detected_trashs_, self.detected_dirts_) = ([], [])

			if DryCleaningBehavior.containsTrashcanTask(cleaning_tasks):
				Thread(target=self.callEmptyService, args=(srv.START_TRASH_DETECTOR_SERVICE_STR,)).start()

			if DryCleaningBehavior.containsDirtTask(cleaning_tasks):
				Thread(target=self.callEmptyService, args=(srv.START_DIRT_DETECTOR_SERVICE_STR,)).start()

			room_map_data = self.database_handler_.database_.getRoomById(room_id).room_map_data_
			path_follower.setParameters(
				target_poses=path,
				area_map=room_map_data,
				path_tolerance=0.2,
				goal_position_tolerance=0.5,
				goal_angle_tolerance=1.57
			)

			path_follower.setInterruptVar(self.interrupt_var_)
			explorer_thread = Thread(target=path_follower.executeBehavior)
			explorer_thread.start()

			while not path_follower.is_finished:
				self.local_mutex_.acquire()
				if len(self.detected_dirts_) > 0 or len(self.detected_trashs_) > 0:
					path_follower.interruptExecution()
				self.local_mutex_.release()
				rospy.sleep(2)

			explorer_thread.join()

			if self.handleInterrupt() >= 1:
				return

			# todo: mutex necessary?
			for dirt in self.detected_dirts_:
				self.dirtRoutine(room_id=room_id, detected_dirt=dirt)
			for trash in self.detected_trashs_:
				self.trashcanRoutine(room_id=room_id, detected_trash=trash)

			# start again on the current position
			self.printMsg("Result is {}".format(path_follower.move_base_path_result_))
			last_visited_index = path_follower.move_base_path_result_.last_visited_index
			self.printMsg('Move stopped at position {}'.format(last_visited_index))
			path = path[last_visited_index:]

		if self.dirt_topic_subscriber_ is not None:
			self.dirt_topic_subscriber_.unregister()
		if self.trash_topic_subscriber_ is not None:
			self.trash_topic_subscriber_.unregister()

		# Checkout the completed room
		self.checkoutRoom(room_id=room_id)

	def getCheckpointForRoomId(self, room_id):
		room_counter = 0
		for checkpoint in self.sequencing_result_.checkpoints:
			for _ in checkpoint.room_indices:
				current_room_id = self.mapping_[room_counter]
				if current_room_id == room_id:
					return checkpoint
				room_counter += 1
		assert False

	# Implemented Behavior
	def executeCustomBehavior(self):
		self.move_base_handler_ = move_base_behavior.MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_,
																		srv.MOVE_BASE_SERVICE_STR)
		self.tool_changer_ = tool_changing_behavior.ToolChangingBehavior("ToolChangingBehavior", self.interrupt_var_)
		self.trolley_mover_ = trolley_movement_behavior.TrolleyMovementBehavior("TrolleyMovingBehavior",
																				self.interrupt_var_)
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
