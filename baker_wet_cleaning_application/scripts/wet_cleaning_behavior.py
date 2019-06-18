#!/usr/bin/env python

from abstract_cleaning_behavior import AbstractCleaningBehavior
from move_base_path_behavior import MoveBasePathBehavior
from move_base_wall_follow_behavior import MoveBaseWallFollowBehavior

import rospy
from geometry_msgs.msg import Quaternion
import ipa_building_msgs.srv
import std_srvs.srv
from threading import Thread
import services_params as srv
import dynamic_reconfigure.client

class WetCleaningBehavior(AbstractCleaningBehavior):

	# ========================================================================
	# Description:
	# Handles the wet cleaning process (i.e. Floor cleaning)
	# for all rooms provided in a given list
	# ========================================================================

	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, sequencing_result, room_information_in_meter, mapping, robot_frame_id,
					  robot_radius, coverage_radius, field_of_view, field_of_view_origin, use_cleaning_device):
		# Parameters set from the outside
		self.sequencing_result_ = sequencing_result
		self.database_handler_= database_handler
		self.room_information_in_meter_ = room_information_in_meter
		self.sequencing_result_ = sequencing_result
		self.mapping_ = mapping
		self.robot_frame_id_ = robot_frame_id
		self.robot_radius_ = robot_radius
		self.coverage_radius_ = coverage_radius
		self.field_of_view_ = field_of_view
		self.field_of_view_origin_ = field_of_view_origin
		self.use_cleaning_device_ = use_cleaning_device  # hack: cleaning device can be turned off for trade fair show

		self.map_data_ = self.database_handler_.database_.global_map_data_.map_image_
		self.map_resolution_ = self.database_handler_.database_.global_map_data_.map_resolution_
		self.map_origin_ = self.database_handler_.database_.global_map_data_.map_origin_
		self.map_header_frame_id_ = self.database_handler_.database_.global_map_data_.map_header_frame_id_

		self.move_base_path_service_str_ = srv.MOVE_BASE_PATH_SERVICE_STR
		self.room_exploration_service_str_ = srv.ROOM_EXPLORATION_SERVICE_STR

		self.start_cleaning_service_str_ = srv.START_CLEANING_SERVICE_STR
		self.stop_cleaning_service_str_ = srv.STOP_CLEANING_SERVICE_STR

		self.coverage_monitor_dynamic_reconfigure_service_str_ = srv.COVERAGE_MONITOR_DYNAMIC_RECONFIGURE_SERVICE_STR
		self.receive_coverage_image_service_str_ = srv.RECEIVE_COVERAGE_IMAGE_SERVICE_STR
		self.stop_coverage_monitoring_service_str_ = srv.STOP_COVERAGE_MONITORING_SERVICE_STR

		self.move_base_wall_follow_service_str_ = srv.MOVE_BASE_WALL_FOLLOW_SERVICE_STR

	def returnToRobotStandardState(self):
		if self.use_cleaning_device_:
			self.stopCleaningDevice()

	def startCleaningDevice(self):
		assert self.use_cleaning_device_

		self.printMsg("Start cleaning with " + self.start_cleaning_service_str_)
		rospy.wait_for_service(self.start_cleaning_service_str_)
		try:
			req = rospy.ServiceProxy(self.start_cleaning_service_str_, std_srvs.srv.Trigger)
			resp = req()
			print("Start cleaning returned with success status " + str(resp.success))
		except rospy.ServiceException, e:
			print("Service call to " + self.start_cleaning_service_str_ + " failed: %s" % e)

	def stopCleaningDevice(self):
		assert self.use_cleaning_device_

		self.printMsg("Stop cleaning with " + self.stop_cleaning_service_str_)
		rospy.wait_for_service(self.stop_cleaning_service_str_)
		try:
			req = rospy.ServiceProxy(self.stop_cleaning_service_str_, std_srvs.srv.Trigger)
			resp = req()
			print("Stop cleaning returned with success status " + str(resp.success))
		except rospy.ServiceException, e:
			print("Service call to " + self.stop_cleaning_service_str_ + " failed: %s" % e)

	def requestCoverageMapResponse(self, room_map_data):
		self.printMsg("Receive coverage image from coverage monitor " + self.receive_coverage_image_service_str_)
		rospy.wait_for_service(self.receive_coverage_image_service_str_)
		try:
			req = rospy.ServiceProxy(self.receive_coverage_image_service_str_, ipa_building_msgs.srv.CheckCoverage)
			req.input_map = room_map_data
			req.map_resolution = self.map_resolution_
			req.map_origin = self.map_origin_
			req.field_of_view = self.field_of_view_
			req.field_of_view_origin = self.field_of_view_origin_
			req.coverage_radius = self.coverage_radius_
			req.check_for_footprint = False
			req.check_number_of_coverages = False
			self.coverage_map_response_ = req()
			print "Receive coverage image returned with success status " + str(resp.success)
		except rospy.ServiceException, e:
			print "Service call to " + self.receive_coverage_image_service_str_ + " failed: %s" % e

	# coverage_monitor_server: set the robot configuration (robot_radius, coverage_radius, coverage_offset)
	# with dynamic reconfigure and turn on logging of the cleaned path (service "start_coverage_monitoring")
	def startCoverageMonitoring(self):
		try:
			print("Start coverage monitoring")

			coverage_circle_offset_transform_x = 0.5 * (self.field_of_view_[0].x + self.field_of_view_[2].x)
			coverage_circle_offset_transform_y = 0.5 * (self.field_of_view_[0].y + self.field_of_view_[1].y)

			client = dynamic_reconfigure.client.Client(self.coverage_monitor_dynamic_reconfigure_service_str_, timeout=5)

			rospy.wait_for_service(self.coverage_monitor_dynamic_reconfigure_service_str_ + "/set_parameters")
			client.update_configuration({"map_frame": self.map_header_frame_id_, "robot_frame": self.robot_frame_id_,
										 "coverage_radius": self.coverage_radius_,
										 "coverage_circle_offset_transform_x": coverage_circle_offset_transform_x,
										 "coverage_circle_offset_transform_y": coverage_circle_offset_transform_y,
										 "coverage_circle_offset_transform_z": 0.0,
										 "robot_trajectory_recording_active": True})

		except rospy.ServiceException, e:
			print("Dynamic reconfigure request to " + self.coverage_monitor_dynamic_reconfigure_service_str_ + " failed: %s" % e)

	def stopCoverageMonitoring(self):
		self.printMsg("Stop coverage monitoring with " + self.stop_coverage_monitoring_service_str_)
		rospy.wait_for_service(self.stop_coverage_monitoring_service_str_)
		try:
			req = rospy.ServiceProxy(self.stop_coverage_monitoring_service_str_, std_srvs.srv.Trigger)
			resp = req()
			print("Stop coverage monitoring returned with success status " + str(resp.success))
		except rospy.ServiceException, e:
			print("Service call to " + self.stop_coverage_monitoring_service_str_ + " failed: %s" % e)

	def executeCustomBehaviorInRoomId(self, room_id):

		self.printMsg('Starting Wet Cleaning of room ID {}'.format(room_id))

		starting_position = self.room_information_in_meter_[room_id].room_center
		self.move_base_handler_.setParameters(
			goal_position=starting_position,
			goal_orientation=Quaternion(x=0., y=0., z=0., w=1.),
			header_frame_id='base_link'
		)

		thread_move_to_the_room = Thread(target=self.move_base_handler_.executeBehavior)
		thread_move_to_the_room.start()

		path = self.computeCoveragePath(room_id=room_id)

		if len(path) == 0 or self.handleInterrupt() >= 1:
			return

		if self.use_cleaning_device_:
			self.startCleaningDevice()

		self.startCoverageMonitoring()

		path_follower = MoveBasePathBehavior("MoveBasePathBehavior_PathFollowing", self.interrupt_var_,
											 self.move_base_path_service_str_)

		thread_move_to_the_room.join()
		if self.move_base_handler_.failed():
			self.printMsg('Room center is not accessible. Failed to clean room {}'.format(room_id))
			return

		room_map_data = self.database_handler_.database_.getRoomById(room_id).room_map_data_
		path_follower.setParameters(
			target_poses=path,
			area_map=room_map_data,
			path_tolerance=0.2,
			goal_position_tolerance=0.5,
			goal_angle_tolerance=1.57
		)

		path_follower.setInterruptVar(self.interrupt_var_)
		path_follower.executeBehavior()

		if self.handleInterrupt() >= 1:
			return

		self.requestCoverageMapResponse(room_map_data)

		if self.handleInterrupt() >= 1:
			return

		wall_follower = MoveBaseWallFollowBehavior("MoveBaseWallFollowBehavior", self.interrupt_var_, self.move_base_wall_follow_service_str_)
		wall_follower.setParameters(
			map=self.map_data_,
			area_map=room_map_data,
			coverage_map=room_map_data,  # todo: self.coverage_map_response_.coverage_map,
			map_resolution=self.map_resolution_,
			map_origin=self.map_origin_,
			path_tolerance=0.2,
			goal_position_tolerance=0.3,
			goal_angle_tolerance=1.57,
			target_wall_distance=0.1,
			wall_following_off_traveling_distance_threshold=0.8
		)
		wall_follower.executeBehavior()

		if self.handleInterrupt() >= 1:
			return

		self.stopCoverageMonitoring()

		if self.handleInterrupt() >= 1:
			return

		if self.use_cleaning_device_:
			self.stopCleaningDevice()

		# Checkout the completed room
		self.checkoutRoom(room_id=room_id)
