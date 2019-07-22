#!/usr/bin/env python

from move_base_behavior import MoveBaseBehavior
from room_exploration_behavior import RoomExplorationBehavior
from tool_changing_behavior import ToolChangingBehavior
from trolley_movement_behavior import TrolleyMovementBehavior
from behavior_container import BehaviorContainer

from utils import getCurrentRobotPosition
from geometry_msgs.msg import Pose2D, Quaternion
import ipa_building_msgs.srv
import std_srvs.srv
import rospy
import services_params as srv
from math import pi
import dynamic_reconfigure.client
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from threading import Thread

class AbstractCleaningBehavior(BehaviorContainer):

	# ========================================================================
	# Description:
	# Hangles a cleaning process (can be a dry or a wet cleaning_behavior)
	# for all rooms provided in a given list
	# ========================================================================

	def __init__(self, behavior_name, interrupt_var):
		super(AbstractCleaningBehavior, self).__init__(behavior_name, interrupt_var)
		(self.move_base_handler_, self.tool_changer_, self.trolley_mover_) = (None, None, None)
		self.room_exploration_service_str_ = srv.ROOM_EXPLORATION_SERVICE_STR
		self.move_base_path_service_str_ = srv.MOVE_BASE_PATH_SERVICE_STR

		self.coverage_monitor_dynamic_reconfigure_service_str_ = srv.COVERAGE_MONITOR_DYNAMIC_RECONFIGURE_SERVICE_STR
		self.receive_coverage_image_service_str_ = srv.RECEIVE_COVERAGE_IMAGE_SERVICE_STR
		self.stop_coverage_monitoring_service_str_ = srv.STOP_COVERAGE_MONITORING_SERVICE_STR
		self.start_coverage_monitoring_service_str_ = srv.START_COVERAGE_MONITORING_SERVICE_STR
		self.reset_coverage_monitoring_service_str_ = srv.RESET_COVERAGE_MONITORING_SERVICE_STR

		self.coverage_map_ = None

		self.thread_move_to_the_room = None

	def setCommonParameters(self, database_handler, sequencing_result, mapping, coverage_radius, field_of_view,
					   field_of_view_origin, room_information_in_meter, robot_radius, robot_frame_id):
		self.database_handler_ = database_handler
		self.sequencing_result_ = sequencing_result
		self.mapping_ = mapping
		self.coverage_radius_ = coverage_radius
		self.field_of_view_ = field_of_view
		self.room_information_in_meter_ = room_information_in_meter
		self.field_of_view_origin_ = field_of_view_origin
		self.robot_radius_ = robot_radius
		self.robot_frame_id_ = robot_frame_id

		self.map_data_ = self.database_handler_.database_.global_map_data_.map_image_
		self.map_resolution_ = self.database_handler_.database_.global_map_data_.map_resolution_
		self.map_origin_ = self.database_handler_.database_.global_map_data_.map_origin_
		self.map_header_frame_id_ = self.database_handler_.database_.global_map_data_.map_header_frame_id_

	def callService(self, service_name, service_message):
		print("Call for service {}".format(service_name))
		rospy.wait_for_service(service_name)
		try:
			req = rospy.ServiceProxy(service_name, service_message)
			req()
		except rospy.ServiceException, e:
			print("Service call to {} failed: {}".format(service_name, e))

	# coverage_monitor_server: set the robot configuration (robot_radius, coverage_radius, coverage_offset)
	# with dynamic reconfigure and turn on logging of the cleaned path (service "start_coverage_monitoring")
	def initAndStartCoverageMonitoring(self):
		try:
			print("Init and start coverage monitoring")

			coverage_circle_offset_transform_x = 0.5 * (self.field_of_view_[0].x + self.field_of_view_[2].x)
			coverage_circle_offset_transform_y = 0.5 * (self.field_of_view_[0].y + self.field_of_view_[1].y)

			client = dynamic_reconfigure.client.Client(self.coverage_monitor_dynamic_reconfigure_service_str_, timeout=5)

			rospy.wait_for_service(self.coverage_monitor_dynamic_reconfigure_service_str_ + "/set_parameters")
			client.update_configuration({
				"map_frame": self.map_header_frame_id_, "robot_frame": self.robot_frame_id_,
				"coverage_radius": self.coverage_radius_,
				"coverage_circle_offset_transform_x": coverage_circle_offset_transform_x,
				"coverage_circle_offset_transform_y": coverage_circle_offset_transform_y,
				"coverage_circle_offset_transform_z": 0.0,
				"robot_trajectory_recording_active": True
			})

		except rospy.ServiceException, e:
			print("Dynamic reconfigure request to " + self.coverage_monitor_dynamic_reconfigure_service_str_ + " failed: %s" % e)

	def startCoverageMonitoring(self):
		self.callService(self.start_coverage_monitoring_service_str_, std_srvs.srv.Trigger)

	def stopCoverageMonitoring(self):
		self.callService(self.stop_coverage_monitoring_service_str_, std_srvs.srv.Trigger)

	def resetCoverageMonitoring(self):
		self.callService(self.reset_coverage_monitoring_service_str_, std_srvs.srv.Trigger)

	def requestCoverageMapResponse(self, room_id):
		self.coverage_map_service_ = srv.RECEIVE_COVERAGE_IMAGE_SERVICE_STR

		area_map = self.database_handler_.database_.getRoomById(room_id).room_map_data_
		self.printMsg("Receive coverage image from coverage monitor " + self.coverage_map_service_)
		rospy.wait_for_service(self.coverage_map_service_)
		try:
			coverage_image_getter = rospy.ServiceProxy(self.coverage_map_service_, ipa_building_msgs.srv.CheckCoverage)
			request = ipa_building_msgs.srv.CheckCoverageRequest()
			request.input_map = area_map
			request.map_resolution = self.map_resolution_
			request.map_origin = self.map_origin_
			request.field_of_view = self.field_of_view_
			request.field_of_view_origin = self.field_of_view_origin_
			request.coverage_radius = self.coverage_radius_
			request.check_for_footprint = False
			request.check_number_of_coverages = False
			return coverage_image_getter(request).coverage_map
		except rospy.ServiceException, e:
			print ("Service call to " + self.coverage_map_service_ + " failed: %s" % e)

	def checkAndComputeCoverageRatio(self, room_id, coverage_map=None):
		map_image = self.database_handler_.database_.getRoomById(room_id).room_map_data_
		map_image = CvBridge().imgmsg_to_cv2(map_image, desired_encoding="passthrough")

		if coverage_map is None:
			coverage_map = self.requestCoverageMapResponse(room_id)
			coverage_map = CvBridge().imgmsg_to_cv2(coverage_map, desired_encoding="passthrough")

		ratio_cleaned = float(np.sum(coverage_map))/np.sum(map_image)

		print("CLEANED {}".format(ratio_cleaned))
		assert(ratio_cleaned <= 1)
		#  if ratio_cleaned < 0.9:
		#     raise RuntimeWarning('Only {}% of room {} cleaned'.format(100*ratio_cleaned, room_id))

		return ratio_cleaned

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

	def checkoutRoom(self, room_id, cleaning_method, nb_found_dirtspots=0, nb_found_trashcans=0, coverage_ratio=0):

		room = self.database_handler_.database_.getRoomById(room_id)

		self.database_handler_.checkoutCompletedRoom(room, assignment_type=cleaning_method - 1)

		if -1 in room.open_cleaning_tasks_ and cleaning_method == 1:  # trash
			self.database_handler_.checkoutCompletedRoom(room, assignment_type=-1)

		self.database_handler_.addLogEntry(
			room_id=room_id,
			status=1,  # 1=Completed
			cleaning_task=cleaning_method,
			found_dirtspots=nb_found_dirtspots,
			found_trashcans=nb_found_trashcans,
			cleaned_surface_area=coverage_ratio*room.room_surface_area_,
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

	def startMoveToTheRoom(self, room_id):
		starting_position = self.room_information_in_meter_[room_id].room_center
		self.move_base_handler_.setParameters(
			goal_position=starting_position,
			goal_orientation=Quaternion(x=0., y=0., z=0., w=1.),
			goal_angle_tolerance=2*pi,
			goal_position_tolerance=0.5
		)

		self.thread_move_to_the_room = Thread(target=self.move_base_handler_.executeBehavior)
		self.thread_move_to_the_room.start()

	def waitMoveToTheRoom(self):
		assert self.thread_move_to_the_room is not None
		self.thread_move_to_the_room.join()

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
				goal_position_tolerance=0.5,
				goal_angle_tolerance=2*pi
			)
			self.move_base_handler_.executeBehavior()
			for _ in checkpoint.room_indices:
				current_room_id = self.mapping_[room_counter]
				self.executeCustomBehaviorInRoomId(room_id=current_room_id)
				room_counter += 1
