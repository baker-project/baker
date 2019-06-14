#!/usr/bin/env python

import dynamic_reconfigure.client
import ipa_building_msgs.srv
import rospy
import std_srvs.srv
from geometry_msgs.msg import Pose2D

import move_base_behavior
import move_base_path_behavior
import move_base_wall_follow_behavior
import room_exploration_behavior
import services_params as srv
import behavior_container


class RoomWetFloorCleaningBehavior(behavior_container.BehaviorContainer):

	# ========================================================================
	# Description:
	# Class which contains the behavior of cleaning the floor of a single room
	# This class does not require database elements
	# ========================================================================
		
	# Method for setting parameters for the behavior
	def setParameters(self, room_map_data, room_center, map_data, map_resolution, map_origin, map_header_frame_id, robot_frame_id, robot_radius, coverage_radius,
					field_of_view, field_of_view_origin, use_cleaning_device):
		# Parameters set from the outside
		self.room_map_data_ = room_map_data
		self.room_center_ = room_center
		self.map_data_ = map_data
		self.map_resolution_ = map_resolution
		self.map_origin_ = map_origin
		self.map_header_frame_id_ = map_header_frame_id
		self.robot_frame_id_ = robot_frame_id
		self.robot_radius_ = robot_radius
		self.coverage_radius_ = coverage_radius
		self.field_of_view_ = field_of_view
		self.field_of_view_origin_ = field_of_view_origin
		self.use_cleaning_device_ = use_cleaning_device	 # todo: hack: cleaning device can be turned off for trade fair show
		# Parameters set autonomously
		self.room_exploration_service_str_ = srv.ROOM_EXPLORATION_SERVICE_STR
		self.move_base_path_service_str_ = srv.MOVE_BASE_PATH_SERVICE_STR
		self.move_base_wall_follow_service_str_ = srv.MOVE_BASE_WALL_FOLLOW_SERVICE_STR
		self.move_base_service_str_ = srv.MOVE_BASE_SERVICE_STR
		self.start_cleaning_service_str_ = srv.START_CLEANING_SERVICE_STR
		self.stop_cleaning_service_str_ = srv.STOP_CLEANING_SERVICE_STR
		self.coverage_monitor_dynamic_reconfigure_service_str_ = srv.COVERAGE_MONITOR_DYNAMIC_RECONFIGURE_SERVICE_STR
		self.stop_coverage_monitoring_service_str_ = srv.STOP_COVERAGE_MONITORING_SERVICE_STR
		self.receive_coverage_image_service_str_ = srv.RECEIVE_COVERAGE_IMAGE_SERVICE_STR


	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone

		# todo: hack: cleaning device can be turned off for trade fair show
		if not self.use_cleaning_device_:
			return

		# baker_brush_cleaning_module_interface: turn off the cleaning device (service "stop_brush_cleaner")
		self.printMsg("Stop cleaning with " + self.stop_cleaning_service_str_)
		rospy.wait_for_service(self.stop_cleaning_service_str_)
		try:
			req = rospy.ServiceProxy(self.stop_cleaning_service_str_, std_srvs.srv.Trigger)
			resp = req()
			print "Stop cleaning returned with success status " + str(resp.success)
		except rospy.ServiceException, e:
			print "Service call to " + self.stop_cleaning_service_str_ + " failed: %s" % e


	# Implemented Behavior
	def executeCustomBehavior(self):
		self.move_base_handler_ = move_base_behavior.MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_, self.move_base_service_str_)
		self.room_explorer_ = room_exploration_behavior.RoomExplorationBehavior("RoomExplorationBehavior", self.interrupt_var_, self.room_exploration_service_str_)
		self.path_follower_ = move_base_path_behavior.MoveBasePathBehavior("MoveBasePathBehavior_PathFollowing", self.interrupt_var_, self.move_base_path_service_str_)
		self.wall_follower_ = move_base_wall_follow_behavior.MoveBaseWallFollowBehavior("MoveBaseWallFollowBehavior", self.interrupt_var_, self.move_base_wall_follow_service_str_)
		#self.trashcan_emptier_ = trashcan_emptying_behavior.TrashcanEmptyingBehavior("TrashcanEmptyingBehavior", self.interrupt_var_)
		
		# Room exploration
		"""
		For room exploration:
		map_resolution = self.map_resolution_
		map_origin = self.map_origin_
		robot_radius = 0.325
		coverage_radius = 0.25
		field_of_view = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364), Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)] # this field of view represents the off-center iMop floor wiping device
		starting_position = Pose2D(x=1., y=0., theta=0.)
		planning_mode = 2
		"""
		self.room_explorer_.setParameters(
			input_map=self.room_map_data_,
			map_resolution=self.map_resolution_,
			map_origin=self.map_origin_,
			robot_radius=self.robot_radius_,
			coverage_radius=self.coverage_radius_,
			field_of_view=self.field_of_view_,		# this field of view represents the off-center iMop floor wiping device
			field_of_view_origin=self.field_of_view_origin_,
			starting_position=Pose2D(x=self.room_center_.x, y=self.room_center_.y, theta=0.),	# todo: determine current robot position
			planning_mode=2
		)
		self.room_explorer_.executeBehavior()

		# If no trajectory was created - move on to next room
		if len(self.room_explorer_.exploration_result_.coverage_path_pose_stamped) == 0:
			return
			
		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		#rospy.sleep(20)
		#continue

		# Robot movement into next room
		"""
		For movement to room:
		goal_position = self.room_center_
		goal_orientation = Quaternion(x=0., y=0., z=0., w=0.)
		header_frame_id = 'base_link'
		"""
		# todo: hack: reactivate this code
		"""
		self.printMsg("Moving to room_center in meter=" + str(self.room_center_))
		self.move_base_handler_.setParameters(
			self.room_center_,
			Quaternion(x=0., y=0., z=0., w=1.),
			'base_link'
			)
		self.move_base_handler_.executeBehavior()
		"""

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# baker_brush_cleaning_module_interface: turn on the cleaning device (service "start_brush_cleaner")
		if self.use_cleaning_device_: # todo: hack: cleaning device can be turned off for trade fair show
			self.printMsg("Start cleaning with " + self.start_cleaning_service_str_)
			rospy.wait_for_service(self.start_cleaning_service_str_)
			try:
				req = rospy.ServiceProxy(self.start_cleaning_service_str_, std_srvs.srv.Trigger)
				resp = req()
				print "Start cleaning returned with success status " + str(resp.success)
			except rospy.ServiceException, e:
				print "Service call to " + self.start_cleaning_service_str_ + " failed: %s" % e

		# coverage_monitor_server: set the robot configuration (robot_radius, coverage_radius, coverage_offset) with dynamic reconfigure
		#                          and turn on logging of the cleaned path (service "start_coverage_monitoring")
		try:
			print "Calling dynamic reconfigure at the coverage_monitor_server to set robot radius, coverage_radius, and coverage offset and start coverage monitoring."
			client = dynamic_reconfigure.client.Client(self.coverage_monitor_dynamic_reconfigure_service_str_, timeout=5)
			rospy.wait_for_service(self.coverage_monitor_dynamic_reconfigure_service_str_ + "/set_parameters")
			client.update_configuration({"map_frame":self.map_header_frame_id_, "robot_frame":self.robot_frame_id_,
										"coverage_radius":self.coverage_radius_,
										"coverage_circle_offset_transform_x":0.5*(self.field_of_view_[0].x+self.field_of_view_[2].x),
										"coverage_circle_offset_transform_y":0.5*(self.field_of_view_[0].y+self.field_of_view_[1].y),
										"coverage_circle_offset_transform_z":0.0,
										"robot_trajectory_recording_active":True})
		except rospy.ServiceException, e:
			print "Dynamic reconfigure request to " + self.coverage_monitor_dynamic_reconfigure_service_str_ + " failed: %s" % e
			# todo (rmb-ma). no return?

		# Explored path follow
		"""
		For path follow movement:
		target_poses = exploration_result.coverage_path_pose_stamped
		area_map = self.room_map_data_
		path_tolerance = 0.2
		goal_position_tolerance = 0.5
		goal_angle_tolerance = 1.57
		"""
		self.path_follower_.setParameters(
			target_poses=self.room_explorer_.exploration_result_.coverage_path_pose_stamped,
			area_map=self.room_map_data_,
			path_tolerance=0.2,
			goal_position_tolerance=0.5,
			goal_angle_tolerance=1.57
		)
		# todo (rmb-ma) uncomment
		#self.path_follower_.executeBehavior()

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Wall follow
		"""
		For wall following movement:
		target_poses = exploration_result.coverage_path_pose_stamped
		path_tolerance = 0.2
		goal_position_tolerance = 0.4
		goal_angle_tolerance = 3.14
		"""
		# receive coverage map from coverage monitor
		self.printMsg("Receive coverage image from coverage monitor " + self.receive_coverage_image_service_str_)
		rospy.wait_for_service(self.receive_coverage_image_service_str_)
		try:
			req = rospy.ServiceProxy(self.receive_coverage_image_service_str_, ipa_building_msgs.srv.CheckCoverage)
			req.input_map = self.room_map_data_
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
			# todo (rmb-ma). No return?

		self.wall_follower_.setParameters(
			map=self.map_data_,
			area_map=self.room_map_data_,
			coverage_map=self.room_map_data_,	# todo: self.coverage_map_response_.coverage_map,
			map_resolution=self.map_resolution_,
			map_origin=self.map_origin_,
			path_tolerance=0.2,
			goal_position_tolerance=0.3,
			goal_angle_tolerance=1.57,
			target_wall_distance=0.1,
			wall_following_off_traveling_distance_threshold=0.8
		)
		self.wall_follower_.executeBehavior()

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# coverage_monitor_server.cpp: turn off logging of the cleaned path (service "stop_coverage_monitoring")
		self.printMsg("Stop coverage monitoring with " + self.stop_coverage_monitoring_service_str_)
		rospy.wait_for_service(self.stop_coverage_monitoring_service_str_)
		try:
			req = rospy.ServiceProxy(self.stop_coverage_monitoring_service_str_, std_srvs.srv.Trigger)
			resp = req()
			print "Stop coverage monitoring returned with success status " + str(resp.success)
		except rospy.ServiceException, e:
			print "Service call to " + self.stop_coverage_monitoring_service_str_ + " failed: %s" % e


		# baker_brush_cleaning_module_interface: turn off the cleaning device (service "stop_brush_cleaner")
		if self.use_cleaning_device_: # todo: hack: cleaning device can be turned off for trade fair show
			self.printMsg("Stop cleaning with " + self.stop_cleaning_service_str_)
			rospy.wait_for_service(self.stop_cleaning_service_str_)
			try:
				req = rospy.ServiceProxy(self.stop_cleaning_service_str_, std_srvs.srv.Trigger)
				resp = req()
				print "Stop cleaning returned with success status " + str(resp.success)
			except rospy.ServiceException, e:
				print "Service call to " + self.stop_cleaning_service_str_ + " failed: %s" % e
