#!/usr/bin/env python

from abstract_cleaning_behavior import AbstractCleaningBehavior
from move_base_path_behavior import MoveBasePathBehavior
from move_base_wall_follow_behavior import MoveBaseWallFollowBehavior

from geometry_msgs.msg import Quaternion
import std_srvs.srv
from threading import Thread
import services_params as srv
from cv_bridge import CvBridge, CvBridgeError
from math import pi
import cv2

class WetCleaningBehavior(AbstractCleaningBehavior):

	# ========================================================================
	# Description:
	# Handles the wet cleaning process (i.e. Floor cleaning)
	# for all rooms provided in a given list
	# ========================================================================

	def __init__(self, behavior_name, interrupt_var):
		super(WetCleaningBehavior, self).__init__(behavior_name, interrupt_var)
		self.start_cleaning_service_str_ = srv.START_CLEANING_SERVICE_STR
		self.stop_cleaning_service_str_ = srv.STOP_CLEANING_SERVICE_STR

		self.move_base_wall_follow_service_str_ = srv.MOVE_BASE_WALL_FOLLOW_SERVICE_STR

	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, sequencing_result, room_information_in_meter, mapping, robot_frame_id,
					  robot_radius, coverage_radius, field_of_view, field_of_view_origin, use_cleaning_device):
		self.setCommonParameters(
			database_handler=database_handler,
			sequencing_result=sequencing_result,
			room_information_in_meter=room_information_in_meter,
			mapping=mapping,
			robot_radius=robot_radius,
			coverage_radius=coverage_radius,
			field_of_view_origin=field_of_view_origin,
			field_of_view=field_of_view,
			robot_frame_id=robot_frame_id
		)
		# Parameters set from the outside
		self.use_cleaning_device_ = use_cleaning_device  # hack: cleaning device can be turned off for trade fair show

	def returnToRobotStandardState(self):
		if self.use_cleaning_device_:
			self.stopCleaningDevice()

	def callTriggerService(self, service_name):
		self.callService(service_name, service_message=std_srvs.srv.Trigger)

	def startCleaningDevice(self):
		assert self.use_cleaning_device_
		self.callTriggerService(self.start_cleaning_service_str_)

	def stopCleaningDevice(self):
		assert self.use_cleaning_device_
		self.callTriggerService(self.stop_cleaning_service_str_)

	def executeCustomBehaviorInRoomId(self, room_id):
		self.printMsg('Starting Wet Cleaning of room ID {}'.format(room_id))
		self.startMoveToTheRoom(room_id)

		path = self.computeCoveragePath(room_id=room_id)

		if len(path) == 0 or self.handleInterrupt() >= 1:
			return

		self.waitMoveToTheRoom()

		if self.move_base_handler_.failed():
			self.printMsg('Room center is not accessible. Failed to clean room {}'.format(room_id))
			return

		if self.use_cleaning_device_:
			self.startCleaningDevice()

		path_follower = MoveBasePathBehavior("MoveBasePathBehavior_PathFollowing", self.interrupt_var_,
											 self.move_base_path_service_str_)

		self.resetCoverageMonitoring()
		self.initAndStartCoverageMonitoring()

		room_map_data = self.database_handler_.database_.getRoomById(room_id).room_map_data_
		path_follower.setParameters(
			target_poses=path,
			area_map=room_map_data,
			path_tolerance=0.2,
			goal_position_tolerance=0.5,
			goal_angle_tolerance=1.57
		)

		path_follower.setInterruptVar(self.interrupt_var_)
		# To don't execute the path follower, don't forget to comment the if path_follower.failed()
		path_follower.executeBehavior()

		if path_follower.failed():
			self.printMsg('Error in path following. Failed to clean room {}'.format(room_id))
			return
		if self.handleInterrupt() >= 1:
			return

		coverage_map = self.requestCoverageMapResponse(room_id)
		coverage_map = CvBridge().imgmsg_to_cv2(coverage_map, desired_encoding="passthrough")
		self.resetCoverageMonitoring()

		wall_follower = MoveBaseWallFollowBehavior("MoveBaseWallFollowBehavior", self.interrupt_var_, self.move_base_wall_follow_service_str_)

		wall_follower.setParameters(
			map=self.map_data_,
			area_map=room_map_data,
			coverage_map_service=self.receive_coverage_image_service_str_,
			map_resolution=self.map_resolution_,
			map_origin=self.map_origin_,
			path_tolerance=0.2,
			goal_position_tolerance=0.3,
			goal_angle_tolerance=1.57,
			target_wall_distance=0.15,
			wall_following_off_traveling_distance_threshold=0.8,
			field_of_view_origin=self.field_of_view_origin_,
			field_of_view=self.field_of_view_,
			coverage_radius=self.coverage_radius_
		)

		print('wall_follower.executeBehavior ...')

		wall_follower.executeBehavior()

		if self.handleInterrupt() >= 1:
			return

		if self.handleInterrupt() >= 1:
			return

		if self.use_cleaning_device_:
			self.stopCleaningDevice()

		# Checkout the completed room
		wall_coverage_map = self.requestCoverageMapResponse(room_id)
		wall_coverage_map = CvBridge().imgmsg_to_cv2(wall_coverage_map, desired_encoding="passthrough")
		coverage_map = cv2.add(wall_coverage_map, coverage_map)
		coverage_ratio = self.checkAndComputeCoverageRatio(room_id, coverage_map=coverage_map)
		self.stopCoverageMonitoring()
		self.checkoutRoom(room_id=room_id, cleaning_method=2, coverage_ratio=coverage_ratio)
