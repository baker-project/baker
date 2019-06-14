#!/usr/bin/env python

import rospy
from cob_map_accessibility_analysis.srv import CheckPerimeterAccessibility, CheckPerimeterAccessibilityRequest
from geometry_msgs.msg import Pose2D, Pose
from math import pi

from move_base_behavior import MoveBaseBehavior
import behavior_container


class DirtRemovingBehavior(behavior_container.BehaviorContainer):

	# ========================================================================
	# Description:
	# Class which contains the behavior for removing dirt spots
	# > Go to the dirt location
	# > Clean
	# ========================================================================

	def __init__(self, behavior_name, interrupt_var, move_base_service_str, map_accessibility_service_str):
		super(DirtRemovingBehavior, self).__init__(behavior_name, interrupt_var)
		self.move_base_handler_ = MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_, move_base_service_str)
		self.dirt_position_ = None
		self.map_accessibility_service_str_ = map_accessibility_service_str


	# Method for setting parameters for the behavior
	def setParameters(self, dirt_position):
		self.dirt_position_ = dirt_position


	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	def computeAccessiblePoses(self):
		accessibility_checker = rospy.ServiceProxy(self.map_accessibility_service_str_, CheckPerimeterAccessibility)

		request = CheckPerimeterAccessibilityRequest()

		center = Pose2D()
		(center.x, center.y) = (self.dirt_position_.x, self.dirt_position_.y)
		center.theta = 0
		request.center = center

		request.radius = 1  # todo (rmb-ma) use the value from the detector
		request.rotational_sampling_step = 0.3
		response = accessibility_checker(request)

		accessible_poses = response.accessible_poses_on_perimeter
		return accessible_poses

	# todo (rmb-ma). find the best accessible location
	@staticmethod
	def computeBestPose(accessible_locations):
		assert(len(accessible_locations) > 0)
		best_pose2d = accessible_locations[0]
		best_pose3d = Pose()
		(best_pose3d.position.x, best_pose3d.position.y) = (best_pose2d.x, best_pose2d.y)
		best_pose3d.orientation.z = best_pose2d.theta
		return best_pose3d

	def removeDirt(self):
		# todo (rmb-ma)
		rospy.sleep(2.)

	def checkoutDirt(self):
		# todo (rmb-ma)
		rospy.sleep(2.)

	# Implemented Behavior
	def executeCustomBehavior(self):
		assert(self.dirt_position_ is not None)

		self.printMsg("Removing dirt located on ({}, {})".format(self.dirt_position_.x, self.dirt_position_.y))

		self.printMsg("> Computing the accessible poses")
		accessible_poses2d = self.computeAccessiblePoses()
		if len(accessible_poses2d) == 0:
			# todo - handler (rmb-ma)
			self.printMsg("No accessible poses found.")
			return
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Todo. Computing the best accessible pose.")
		best_pose3d = self.computeBestPose(accessible_poses2d)
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Moving to an accessible pose")
		self.move_base_handler_.setParameters(
			goal_position=best_pose3d.position,
			goal_orientation=best_pose3d.orientation,
			header_frame_id='base_link'
		)
		self.move_base_handler_.executeBehavior()
		if self.handleInterrupt() >= 1:
			return
		if self.move_base_handler_.failed():
			self.printMsg('Room center is not accessible. Failed to for removing dirt ({}, {})'.format(self.dirt_position_.x, self.dirt_position_.y))
			self.state_ = 4
			return

		self.printMsg("> Todo. Remove the dirt")
		self.removeDirt()
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Todo. Checkout the dirt")
		self.checkoutDirt()
