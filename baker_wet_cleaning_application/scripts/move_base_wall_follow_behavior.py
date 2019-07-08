#!/usr/bin/env python

import actionlib
import rospy
from scitos_msgs.msg import MoveBaseWallFollowAction
from scitos_msgs.msg import MoveBaseWallFollowGoal
import ipa_building_msgs.srv
import behavior_container
from cv_bridge import CvBridge, CvBridgeError
import cv2

class MoveBaseWallFollowBehavior(behavior_container.BehaviorContainer):

	#  ========================================================================
	# Description:
	# Class which contains the behavior for making the robot follow along
	# the walls
	#  ========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		super(MoveBaseWallFollowBehavior, self).__init__(behavior_name, interrupt_var)
		self.service_str_ = service_str

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current position (pose, room)
		# return to standard pose / stop all motors
		pass

	# Method for setting parameters for the behavior
	def setParameters(self, map, area_map, coverage_map_service, map_resolution, map_origin, path_tolerance,
					  goal_position_tolerance, goal_angle_tolerance, target_wall_distance, wall_following_off_traveling_distance_threshold,
					  field_of_view, field_of_view_origin, coverage_radius):
		self.map_ = map		# contains map, map_resolution, map_origin
		self.area_map_ = area_map
		self.coverage_map_service_ = coverage_map_service
		self.map_resolution_ = map_resolution
		self.map_origin_ = map_origin
		self.path_tolerance_ = path_tolerance
		self.goal_position_tolerance_ = goal_position_tolerance
		self.goal_angle_tolerance_ = goal_angle_tolerance
		self.target_wall_distance_ = target_wall_distance
		self.wall_following_off_traveling_distance_threshold_ = wall_following_off_traveling_distance_threshold

		self.field_of_view_ = field_of_view
		self.field_of_view_origin_ = field_of_view_origin
		self.coverage_radius_ = coverage_radius

	# todo rmb-ma avoid copy paste with abstract_cleaning_behavior
	def requestCoverageMapResponse(self):

		self.printMsg("Receive coverage image from coverage monitor " + self.coverage_map_service_)
		rospy.wait_for_service(self.coverage_map_service_)
		try:
			coverage_image_getter = rospy.ServiceProxy(self.coverage_map_service_, ipa_building_msgs.srv.CheckCoverage)
			request = ipa_building_msgs.srv.CheckCoverageRequest()
			request.input_map = self.area_map_
			request.map_resolution = self.map_resolution_
			request.map_origin = self.map_origin_
			request.field_of_view = self.field_of_view_
			request.field_of_view_origin = self.field_of_view_origin_
			request.coverage_radius = self.coverage_radius_
			request.check_for_footprint = False
			request.check_number_of_coverages = False
			self.coverage_map_ = coverage_image_getter(request).coverage_map

			#map_image = CvBridge().imgmsg_to_cv2(self.coverage_map_, desired_encoding="passthrough")
			#cv2.imwrite('tmp.png', map_image)
			#cv2.imshow('my_cover', map_image)

			map = cv2.imread('2.png', 0)
			self.coverage_map_ = CvBridge().cv2_to_imgmsg(map, encoding='mono8')

		except rospy.ServiceException, e:
			print ("Service call to " + self.coverage_map_service_ + " failed: %s" % e)

	def computeNewGoalFromPausedResult(self, prev_action_goal, result):
		self.requestCoverageMapResponse()
		prev_action_goal.coverage_map = self.coverage_map_
		return prev_action_goal

	# Implemented Behavior
	def executeCustomBehavior(self):
		self.requestCoverageMapResponse()

		move_base_goal = MoveBaseWallFollowGoal()
		move_base_goal.map = self.map_
		move_base_goal.area_map = self.area_map_
		move_base_goal.coverage_map = self.coverage_map_
		move_base_goal.map_resolution = self.map_resolution_
		move_base_goal.map_origin = self.map_origin_
		move_base_goal.path_tolerance = self.path_tolerance_
		move_base_goal.goal_position_tolerance = self.goal_position_tolerance_
		move_base_goal.goal_angle_tolerance = self.goal_angle_tolerance_
		move_base_goal.target_wall_distance = self.target_wall_distance_
		move_base_goal.wall_following_off_traveling_distance_threshold = self.wall_following_off_traveling_distance_threshold_
		move_base_client = actionlib.SimpleActionClient(self.service_str_, MoveBaseWallFollowAction)
		self.printMsg("Running move_base_wall_follow action...")
		self.move_base_wall_follow_result_ = self.runAction(move_base_client, move_base_goal)['result']
		self.printMsg("move_base_wall_follow completed.")
