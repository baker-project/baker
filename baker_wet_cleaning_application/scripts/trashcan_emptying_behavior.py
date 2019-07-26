#!/usr/bin/env python

import behavior_container
import rospy
import actionlib

from geometry_msgs.msg import Quaternion
from move_base_behavior import MoveBaseBehavior
from ipa_manipulation_msgs.msg import MoveToAction, MoveToGoal
from cob_map_accessibility_analysis.srv import CheckPerimeterAccessibility, CheckPerimeterAccessibilityRequest

import services_params as srv
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose2D, Pose, Quaternion, Point
from utils import projectToFrame, getCurrentRobotPosition
from math import cos, acos, sqrt

class TrashcanEmptyingBehavior(behavior_container.BehaviorContainer):

	# ========================================================================
	# Description:
	# Class which contains the behavior of trashcan emptying
	# > Go to the trashcan
	# > Take the trashcan
	# > Go to the trolley
	# > Empty the trashcan
	# > Go to the trashcan location
	# > Leave the trashcan
	# ========================================================================

	def __init__(self, behavior_name, interrupt_var, move_base_service_str):
		super(TrashcanEmptyingBehavior, self).__init__(behavior_name, interrupt_var)
		self.move_base_handler_ = MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_, move_base_service_str)
		(self.trolley_position_, self.trashcan_position_) = (None, None)

		self.catch_trashcan_service_str_ = srv.CATCH_TRASHCAN_SERVICE_STR
		self.empty_trashcan_service_str_ = srv.EMPTY_TRASHCAN_SERVICE_STR
		self.leave_trashcan_service_str_ = srv.LEAVE_TRASHCAN_SERVICE_STR
		self.transport_position_service_str_ = srv.TRANSPORT_POSITION_STR
		self.rest_position_service_str_ = srv.REST_POSITION_STR
		self.map_accessibility_service_str_ = srv.MAP_ACCESSIBILITY_SERVICE_STR

	# Method for setting parameters for the behavior
	def setParameters(self, trashcan_pose, trolley_position):
		self.trashcan_pose_ = trashcan_pose
		self.trolley_position_ = trolley_position

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	def moveToGoalPosition(self, goal):
		self.move_base_handler_.setParameters(
			goal_position=goal.position,
			goal_orientation=goal.orientation
		)
		self.move_base_handler_.executeBehavior()

	def executeAction(self, action_name, target_pose=None):
		goal = MoveToGoal()
		if target_pose is not None:
			goal = MoveToGoal()
			goal.target_pos.pose = target_pose
			goal.target_pos.header.frame_id = 'map'

		client = actionlib.SimpleActionClient(action_name, MoveToAction)
		client.wait_for_server()
		client.send_goal(goal)
		client.wait_for_result()
		return client.get_result()

	def emptyTrashcan(self):
		target_pose = Pose()
		target_pose.position = self.trolley_position_
		target_pose.position.z = 1.2

		(robot_position, _, _) = getCurrentRobotPosition()
		delta_x = robot_position[0] - self.trolley_position_.x
		delta_y = robot_position[1] - self.trolley_position_.y

		yaw = acos(delta_x / sqrt(delta_x**2 + delta_y**2))
		if delta_y < 0:
			yaw *= -1
		orientation = quaternion_from_euler(0, 0, yaw)
		target_pose.orientation.x = orientation[0]
		target_pose.orientation.y = orientation[1]
		target_pose.orientation.z = orientation[2]
		target_pose.orientation.w = orientation[3]

		return self.executeAction(self.empty_trashcan_service_str_, target_pose) # todo rmb-ma (not trashcan position)

	def leaveTrashcan(self):
		return self.executeAction(self.leave_trashcan_service_str_, self.trashcan_pose_)

	def transportPosition(self):
		return self.executeAction(self.transport_position_service_str_)

	def restPosition(self):
		return self.executeAction(self.rest_position_service_str_)

	def catchTrashcan(self):
		return self.executeAction(self.catch_trashcan_service_str_, self.trashcan_pose_)

	def computeAccessiblePosesAround(self, position):
		accessibility_checker = rospy.ServiceProxy(self.map_accessibility_service_str_, CheckPerimeterAccessibility)

		request = CheckPerimeterAccessibilityRequest()

		center = Pose2D()
		(center.x, center.y) = (position.x, position.y)
		center.theta = 0
		request.center = center

		request.radius = 1.  # todo (rmb-ma) use the value from the robotic arm
		request.rotational_sampling_step = 0.3
		response = accessibility_checker(request)

		accessible_poses = response.accessible_poses_on_perimeter
		return accessible_poses

	def armCanAccessTrashcan(self, pose2d, orientation):
		robot_pose_theta = pose2d.theta

		orientation = self.trashcan_pose_.orientation
		euler_angles = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
		trashcan_pose_theta = euler_angles[2]

		return cos(trashcan_pose_theta - robot_pose_theta) > 0.85 # todo rmb-ma fit the parameter

	def computeRobotGoalPose(self, position, orientation=None):

		position_in_map = Point()
		position_in_map.x = position.x
		position_in_map.y = position.y
		position_in_map.z = 0
		accessible_poses = self.computeAccessiblePosesAround(position_in_map)
		if orientation is not None:
			accessible_poses = list(filter(lambda pose: self.armCanAccessTrashcan(pose, orientation), accessible_poses))

		if len(accessible_poses) == 0:
			return None # todo rmb-ma handle this case
		accessible_pose = Pose()
		accessible_pose.position.x = accessible_poses[0].x
		accessible_pose.position.y = accessible_poses[0].y
		accessible_pose.position.z = 0.

		orientation = quaternion_from_euler(0, 0, accessible_poses[0].theta)
		accessible_pose.orientation.x = orientation[0]
		accessible_pose.orientation.y = orientation[1]
		accessible_pose.orientation.z = orientation[2]
		accessible_pose.orientation.w = orientation[3]

		return accessible_pose

	# Implemented Behavior
	def executeCustomBehavior(self):
		assert(self.trashcan_pose_ is not None and self.trolley_position_ is not None)
		self.printMsg("Executing trashcan behavior located on ({}, {})".format(self.trashcan_pose_.position.x, self.trashcan_pose_.position.y))

		# todo (rmb-ma): see how we can go there + see the locations to clean it
		print("> Computing robot goal position")
		robot_pose_for_catching_trashcan = self.computeRobotGoalPose(self.trashcan_pose_.position, orientation=self.trashcan_pose_.orientation)

		self.printMsg("> Moving to the trashcan")
		self.moveToGoalPosition(robot_pose_for_catching_trashcan)
		if self.move_base_handler_.failed():
			self.printMsg('Trashcan is not accessible. Failed to for emptying trashcan ({}, {})'.format(self.robot_pose_for_catching_trashcan.position.x, self.robot_pose_for_catching_trashcan.position.y))
			self.state_ = 4
			return
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Catch the trashcan")
		if not self.catchTrashcan().arrived:
			raise RuntimeError('Error occured while catching the trashcan')
			# print('error. error. error')
			# return
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Going to transport position")
		self.transportPosition()
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Computing robot goal position for trolley")
		robot_pose_for_emptying_trashcan = self.computeRobotGoalPose(self.trolley_position_)

		self.printMsg("> Moving to the trolley located on ({}, {})".format(self.trolley_position_.x, self.trolley_position_.y))
		self.moveToGoalPosition(robot_pose_for_emptying_trashcan)
		if self.move_base_handler_.failed():
			self.printMsg('Trolley is not accessible. Failed to for emptying trashcan ({}, {})'.format(robot_pose_for_emptying_trashcan.x, robot_pose_for_emptying_trashcan.y))
			self.state_ = 4
			return
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Todo. Empty the trashcan")
		if not self.emptyTrashcan().arrived:
			raise RuntimeError('Error while emptying the trashcan')
			# print('error. error. error')
			# return

		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Going to transport position")
		if not  self.transportPosition().arrived:
			raise RuntimeError('Error while moving to the transport position')
			# print('error. error. error')
			# return
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Going to the trashcan location")
		if self.handleInterrupt() >= 1:
			return
		if self.move_base_handler_.failed():
			self.printMsg('Trashcan location is not accessible. Failed to for emptying trashcan ({}, {})'.format(self.trashcan_position_.position.x, self.trashcan_position_.position.y))
			self.state_ = 4
			return

		self.printMsg("> Todo. Leave the trashcan")
		if not self.leaveTrashcan():
			raise RuntimeError('Error while leaving the trashcan')
			# print('error. error. error')
			# return
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Going to rest position")
		if not self.restPosition():
			raise RuntimeError('Error while moving to the rest position')
			# print('error. error. error')
			# return
		if self.handleInterrupt() >= 1:
			return
