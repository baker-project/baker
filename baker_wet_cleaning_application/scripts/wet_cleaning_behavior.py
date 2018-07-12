#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose2D, Point32, Quaternion
import std_srvs.srv
import dynamic_reconfigure.client
import ipa_building_msgs.srv

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import threading


import behavior_container
import move_base_behavior
import room_exploration_behavior
import move_base_path_behavior
import trolley_movement_behavior
import tool_changing_behavior
import move_base_wall_follow_behavior

class WetCleaningBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Handles the wet cleanig process (i.e. Floor cleaning, Trashcan)
	# for all rooms provided in a given list
	#========================================================================
	# Serivces to be used:
	# room_exploration_service_str_ = 
	#       '/room_exploration_server'
	# move_base_path_service_str_ =
	#		'/move_base_path'
	# move_base_wall_follow_service_str_ =
	#		'/move_base_wall_follow'
	# move_base_service_str_ =
	#		'move_base'
	#========================================================================

		
	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, segmented_map, room_information_in_meter, sequence_data, mapping, robot_frame_id, robot_radius, coverage_radius, field_of_view):
		# Parameters set from the outside
		self.database_handler_= database_handler
		self.segmented_map_ = segmented_map
		self.room_information_in_meter_ = room_information_in_meter
		self.sequence_data_ = sequence_data
		self.mapping_ = mapping
		self.robot_frame_id_ = robot_frame_id
		self.robot_radius_ = robot_radius
		self.coverage_radius_ = coverage_radius
		self.field_of_view_ = field_of_view
		# Parameters set autonomously
		self.room_exploration_service_str_ = '/room_exploration/room_exploration_server'
		self.move_base_path_service_str_ = '/move_base_path'
		self.move_base_wall_follow_service_str_ = '/move_base_wall_follow'
		self.move_base_service_str_ = 'move_base'
		self.start_cleaning_service_str_ = '/brush_cleaning_module_interface/start_brush_cleaner'
		self.stop_cleaning_service_str_ = '/brush_cleaning_module_interface/stop_brush_cleaner'
		self.coverage_monitor_dynamic_reconfigure_service_str_ = '/room_exploration/coverage_monitor_server'
		self.stop_coverage_monitoring_service_str_ = "/room_exploration/coverage_monitor_server/stop_coverage_monitoring"
		self.receive_coverage_image_service_str_ = "/room_exploration/coverage_monitor_server/get_coverage_image"
		self.trolley_movement_service_str_ = ""
		self.tool_changing_service_str_ = ""
		# Get a opencv representation of the segmented image
		self.bridge_ = CvBridge()
		self.opencv_segmented_map_ = self.bridge_.imgmsg_to_cv2(self.segmented_map_, desired_encoding = "passthrough")



	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# nothing to save
		# nothing to be undone
		pass



	# Driving through room and wet cleaning
	def driveCleaningTrajectory(self, room_counter, current_room_index):
		self.printMsg("Moving to next room with current_room_index = " + str(current_room_index))

		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return

		# Room exploration
		"""
		For room exploration:
		input_map = self.getMapSegmentAsImageMsg(current_room_index)
		map_resolution = self.database_handler_.database_.global_map_data_.map_resolution_
		map_origin = self.database_handler_.database_.global_map_data_.map_origin_
		robot_radius = 0.325
		coverage_radius = 0.25
		field_of_view = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364), Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)] # this field of view represents the off-center iMop floor wiping device
		starting_position = Pose2D(x=1., y=0., theta=0.)
		planning_mode = 2
		"""

		current_room_center = self.room_information_in_meter_[current_room_index].room_center
		current_room_map = self.database_handler_.database_.getRoom(self.mapping_.get(current_room_index)).room_map_data_
		self.room_explorer_.setParameters(
			current_room_map,
			self.database_handler_.database_.global_map_data_.map_resolution_,
			self.database_handler_.database_.global_map_data_.map_origin_,
			robot_radius = self.robot_radius_,
			coverage_radius = self.coverage_radius_,
			field_of_view = self.field_of_view_,		# this field of view represents the off-center iMop floor wiping device
			starting_position = Pose2D(x=current_room_center.x, y=current_room_center.y, theta=0.),	# todo: determine current robot position
			planning_mode = 2
		)
		self.room_explorer_.executeBehavior()

		# If no trajectory was created - move on to next room
		if (self.room_explorer_.exploration_result_ != None):
			
			# Interruption opportunity
			if self.handleInterrupt() == 2:
				return

			#rospy.sleep(20)
			#continue

			# Robot movement into next room
			"""
			For movement to room:
			goal_position = self.segmentation_data_.room_information_in_meter[current_room_index].room_center
			goal_orientation = Quaternion(x=0., y=0., z=0., w=0.)
			header_frame_id = 'base_link'
			"""
			# todo: hack: reactivate this code
			"""
			self.printMsg("Moving to room_center in meter=" + str(self.room_information_in_meter_[current_room_index].room_center))
			self.move_base_handler_.setParameters(
				self.room_information_in_meter_[current_room_index].room_center,
				Quaternion(x=0., y=0., z=0., w=1.),
				'base_link'
				)
			self.move_base_handler_.executeBehavior()
			"""
			
			# Interruption opportunity
			if self.handleInterrupt() == 2:
				return
			
			# baker_brush_cleaning_module_interface: turn on the cleaning device (service "start_brush_cleaner")
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
				client.update_configuration({"map_frame":self.database_handler_.database_.global_map_data_.map_header_frame_id_, "robot_frame":self.robot_frame_id_,
											"coverage_radius":self.coverage_radius_,
											"coverage_circle_offset_transform_x":0.5*(self.field_of_view_[0].x+self.field_of_view_[2].x),
											"coverage_circle_offset_transform_y":0.5*(self.field_of_view_[0].y+self.field_of_view_[1].y),
											"coverage_circle_offset_transform_z":0.0,
											"robot_trajectory_recording_active":True})
			except rospy.ServiceException, e:
				print "Dynamic reconfigure request to " + self.coverage_monitor_dynamic_reconfigure_service_str_ + " failed: %s" % e
			
			# Explored path follow
			"""
			For path follow movement:
			target_poses = exploration_result.coverage_path_pose_stamped
			area_map = current_room_map
			path_tolerance = 0.2
			goal_position_tolerance = 0.5
			goal_angle_tolerance = 1.57
			"""
			self.path_follower_.setParameters(
				self.room_explorer_.exploration_result_.coverage_path_pose_stamped,
				current_room_map,
				0.2,
				0.5,
				1.57
			)
			self.path_follower_.executeBehavior()
			
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
			# receive coverage map from coverage monitor
			"""
			self.printMsg("Receive coverage image from coverage monitor " + self.receive_coverage_image_service_str_)
			rospy.wait_for_service(self.receive_coverage_image_service_str_) 
			try:
				req = rospy.ServiceProxy(self.receive_coverage_image_service_str_, ipa_building_msgs.srv.CheckCoverage)
				req.input_map = current_room_map
				req.map_resolution = self.map_data_.map_resolution
				req.map_origin = self.map_data_.map_origin
				req.field_of_view = self.field_of_view_
				req.coverage_radius = self.coverage_radius_
				req.check_for_footprint = False
				req.check_number_of_coverages = False
				self.coverage_map_response_ = req()
				print "Receive coverage image returned with success status " + str(resp.success)
			except rospy.ServiceException, e:
				print "Service call to " + self.receive_coverage_image_service_str_ + " failed: %s" % e
			"""
			'''
			self.wall_follower_.setParameters(
				self.map_data_.map
				current_room_map,
				self.coverage_map_response_.coverage_map,
				0.2,
				0.4,
				1.57
			)
			self.wall_follower_.executeBehavior()
			'''
			
			# Interruption opportunity
			if self.handleInterrupt() == 2:
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
			self.printMsg("Stop cleaning with " + self.stop_cleaning_service_str_)
			rospy.wait_for_service(self.stop_cleaning_service_str_) 
			try:
				req = rospy.ServiceProxy(self.stop_cleaning_service_str_, std_srvs.srv.Trigger)
				resp = req()
				print "Stop cleaning returned with success status " + str(resp.success)
			except rospy.ServiceException, e:
				print "Service call to " + self.stop_cleaning_service_str_ + " failed: %s" % e

			
			# Mark the current room as finished
			self.printMsg("ID of cleaned room: " + str(self.mapping_.get(room_counter)))
			self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), 1)
			self.printMsg(str(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)).open_cleaning_tasks_))



	# Searching for trashcans
	def trashcanRoutine(self, room_counter):
		# ==========================================
		# insert trashcan handling here
		# ==========================================
		self.database_handler_.checkoutCompletedRoom(self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)), -1)



	# Implemented Behavior
	def executeCustomBehavior(self):
		self.move_base_handler_ = move_base_behavior.MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_, self.move_base_service_str_)
		self.room_explorer_ = room_exploration_behavior.RoomExplorationBehavior("RoomExplorationBehavior", self.interrupt_var_, self.room_exploration_service_str_)
		self.path_follower_ = move_base_path_behavior.MoveBasePathBehavior("MoveBasePathBehavior_PathFollowing", self.interrupt_var_, self.move_base_path_service_str_)
		self.wall_follower_ = move_base_wall_follow_behavior.MoveBaseWallFollowBehavior("MoveBaseWallFollowBehavior", self.interrupt_var_, self.move_base_wall_follow_service_str_)
		self.trolley_mover_ = trolley_movement_behavior.TrolleyMovementBehavior("TrolleyMovementBehavior", self.interrupt_var_)
		self.tool_changer_ = tool_changing_behavior.ToolChangingBehavior("ToolChangingBehavior", self.interrupt_var_)


		# TOOL CHANGE ACCORDING TO CLEANING TASK
		# ======================================

		self.tool_changer_.setParameters(self.database_handler_)
		self.tool_changer_.executeBehavior()


		# Room counter index: Needed for mapping of room_indices <--> RoomItem.room_id
		room_counter = 0


		for current_checkpoint_index in range(len(self.sequence_data_.checkpoints)):


			# TROLLEY MOVEMENT TO CHECKPOINT
			# ==============================

			self.trolley_mover_.setParameters(self.database_handler_)
			self.trolley_mover_.executeBehavior()


			for current_room_index in self.sequence_data_.checkpoints[current_checkpoint_index].room_indices:


				# HANDLING OF SELECTED ROOM
				# =========================
				cleaning_tasks = self.database_handler_.database_.getRoom(self.mapping_.get(room_counter)).open_cleaning_tasks_

				cleaning_thread = threading.Thread(target = self.driveCleaningTrajectory(room_counter, current_room_index))
				cleaning_thread.start()
				if ((-1 in cleaning_tasks) == True):
					trashcan_thread = threading.Thread(target = self.trashcanRoutine(room_counter))
					trashcan_thread.start()
				cleaning_thread.join()
				
				# Increment the current room counter index
				room_counter = room_counter + 1