#!/usr/bin/env python

import rospy
import tf
from cob_map_accessibility_analysis.srv import CheckPerimeterAccessibility, CheckPerimeterAccessibilityRequest
from baker_msgs.srv import CleanPattern, CleanPatternRequest
from geometry_msgs.msg import Pose2D, Pose, Quaternion
from utils import getCurrentRobotPosition

from move_base_behavior import MoveBaseBehavior
import behavior_container

from math import pi, sqrt, sin, cos, atan2
from tf.transformations import quaternion_from_euler

class DirtRemovingBehavior(behavior_container.BehaviorContainer):

	# ========================================================================
	# Description:
	# Class which contains the behavior for removing dirt spots
	# > Go to the dirt location
	# > Clean
	# ========================================================================

	def __init__(self, behavior_name, interrupt_var, move_base_service_str, map_accessibility_service_str, clean_pattern_str):
		super(DirtRemovingBehavior, self).__init__(behavior_name, interrupt_var)
		self.move_base_handler_ = MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_, move_base_service_str)
		self.dirt_position_ = None
		self.map_accessibility_service_str_ = map_accessibility_service_str
		self.clean_pattern_str_ = clean_pattern_str


	# Method for setting parameters for the behavior
	def setParameters(self, dirt_position):
		self.dirt_position_ = dirt_position


	# Method for retcomputeBestAccessiblePoseurning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	def computeAccessiblePosesForVacuumCleaner(self):
		accessibility_checker = rospy.ServiceProxy(self.map_accessibility_service_str_, CheckPerimeterAccessibility)

		request = CheckPerimeterAccessibilityRequest()

		center = Pose2D()
		(center.x, center.y) = (self.dirt_position_.x, self.dirt_position_.y)
		center.theta = 0
		request.center = center

		request.radius = 0.8  # todo (rmb-ma) use the value from the cleaning device
		request.rotational_sampling_step = 0.3
		response = accessibility_checker(request)

		accessible_poses = response.accessible_poses_on_perimeter
		return accessible_poses

	@staticmethod
	def computeBestPoseForRobot(accessible_locations):
		assert(len(accessible_locations) > 0)

		(robot_position, robot_rotation, _) = getCurrentRobotPosition()
		(x_robot, y_robot) = (robot_position[0], robot_position[1])

		min_dist = float('inf')
		for pose in accessible_locations:
			(x, y) = (pose.x, pose.y)
			current_dist = (x - x_robot)**2 + (y - y_robot)**2
			if current_dist < min_dist:
				min_dist = current_dist
				best_pose2d = pose

		best_pose3d = Pose()
		(best_pose3d.position.x, best_pose3d.position.y) = (best_pose2d.x, best_pose2d.y)
		best_pose3d.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, best_pose2d.theta))
		return best_pose3d

	@staticmethod
	def normalize(angle):
		return atan2(sin(angle), cos(angle))

	@staticmethod
	def setTheta(pose, angle):
		orientation = quaternion_from_euler(0, 0, angle)
		pose.orientation.x = orientation[0]
		pose.orientation.y = orientation[1]
		pose.orientation.z = orientation[2]
		pose.orientation.w = orientation[3]

	def computeBestPose(self):

		# Setting parameters
		theta_left = 60*pi/180 # angle opening on the left (POSITIVE VALUE between 0 and pi)
		theta_right = 90*pi/180 # angle opening on the right (POSITIVE VALUE between 0 and pi)

		distance_robot_vacuum_cleaner = 0.4 # in meter, on x axis only
		length_vacuum_cleaner_arm = 0.55 # in meter

		max_robot_dirt_distance = distance_robot_vacuum_cleaner + length_vacuum_cleaner_arm # when the robot looks into the dirt

		# Using Al-Kashi theorem (in any triangle with usual notations
		# (a, b, c for distances and alpha, beta, gamma for oposit angles):
		# a^2 = b^2 + c^2 - 2b.c.cos(alpha)
		# (in our case we use b = sqrt(a^2 - c^2 + 2b.c.cos(alpha)))

		robot_dirt_distance_left = sqrt(length_vacuum_cleaner_arm**2 - distance_robot_vacuum_cleaner**2 + 2*length_vacuum_cleaner_arm*distance_robot_vacuum_cleaner*cos(theta_left))
		robot_dirt_distance_right = sqrt(length_vacuum_cleaner_arm**2 - distance_robot_vacuum_cleaner**2 + 2*length_vacuum_cleaner_arm*distance_robot_vacuum_cleaner*cos(theta_right))

		min_robot_dirt_distance = min(robot_dirt_distance_left, robot_dirt_distance_right)

		# For every robot position with a distance to the dirt between min_robot_dirt_distance and max_robot_dirt_distance
		# there exits at least one orientation to clean the dirt

		(robot_position, robot_orientation, robot_rotation) = getCurrentRobotPosition()
		angle_robot = self.normalize(robot_rotation[0]) # between -pi and pi
		(x_robot, y_robot) = (robot_position[0], robot_position[1])
		dx = self.dirt_position_.x - x_robot
		dy = self.dirt_position_.y - y_robot
		current_robot_dirt_distance = sqrt(dx**2 + dy**2)

		angle_robot_dirt = atan2(dy, dx) # between -pi and +pi
		(x_vacuum, y_vacuum) = (x_robot + distance_robot_vacuum_cleaner*cos(angle_robot), y_robot + distance_robot_vacuum_cleaner*sin(angle_robot))
		
		dx = -x_vacuum + self.dirt_position_.x
		dy = -y_vacuum + self.dirt_position_.y

		angle_vacuum_dirt = atan2(dy, dx) # between -pi and +pi

		
		diff_angle = self.normalize(angle_vacuum_dirt - angle_robot)

		best_pose = Pose()
		best_pose.position.x = x_robot
		best_pose.position.y = y_robot
		best_pose.position.z = 0
		self.setTheta(best_pose, angle_robot)
		
		print(-theta_right, diff_angle, theta_left)
		print(min_robot_dirt_distance, current_robot_dirt_distance, max_robot_dirt_distance)
		if min_robot_dirt_distance <= current_robot_dirt_distance and current_robot_dirt_distance <= max_robot_dirt_distance and -theta_right <= diff_angle and diff_angle <= theta_left:
			best_robot_angle = angle_robot
			self.setTheta(best_pose, best_robot_angle)
			#elif diff_angle <= 0: # WARNING - ??
			#	best_robot_angle = angle_robot_dirt + theta_right
			#	print('diff <= 0', best_robot_angle)
			#elif diff_angle > 0:
			#	best_robot_angle = angle_robot_dirt - theta_left
			#	print('diff >= 0', best_robot_angle)
			

		else: # first position in -angle_robot_dirt direction
			#best_pose.position.x = self.dirt_position_.x + max_robot_dirt_distance*cos(-angle_robot_dirt) # pi/2
			#best_pose.position.y = self.dirt_position_.y + max_robot_dirt_distance*sin(-angle_robot_dirt)
			#self.setTheta(best_pose, angle_robot_dirt)
			#print('positon', best_pose.position.x, best_pose.position.y)
			# Help the names are not correct
			accessible_poses = self.computeAccessiblePosesForVacuumCleaner()
			best_pose = self.computeBestPoseForRobot(accessible_poses)
		return best_pose

	def removeDirt(self):
		clean_pattern = rospy.ServiceProxy(self.clean_pattern_str_, CleanPattern)


		(robot_position, robot_orientation, robot_rotation) = getCurrentRobotPosition()
		theta = robot_rotation[0]
		(x_robot, y_robot) = (robot_position[0], robot_position[1])
		distance_robot_vacuum = 0.4
		(x_vacuum, y_vacuum) = (x_robot + distance_robot_vacuum*cos(theta), y_robot + distance_robot_vacuum*sin(theta)) # WARNING DISTANCE HARDCODED
		
		dx = -x_vacuum + self.dirt_position_.x
		dy = -y_vacuum + self.dirt_position_.y

		angle_vacuum_dirt = atan2(dy, dx) - theta # between -pi and +pi

		request = CleanPatternRequest()
		print('vacuum angle', angle_vacuum_dirt*180/pi, dx, dy)
		request.clean_pattern_params.directions = [ angle_vacuum_dirt*180/pi ]
		
		request.clean_pattern_params.repetitions = [ 1 ]
		request.clean_pattern_params.retract = True
		response = clean_pattern(request)

		return response.success

		return True

	def checkoutDirt(self):
		# todo (rmb-ma)
		rospy.sleep(1.)

	# Implemented Behavior
	def executeCustomBehavior(self):
		assert(self.dirt_position_ is not None)

		self.printMsg("Removing dirt located on ({}, {})".format(self.dirt_position_.x, self.dirt_position_.y))

		# The algorithm computes all the accessible poses for the vacuum cleaner
		# (function computeAccessiblePosesForVacuumCleaner)
		# around the dirt (radius hardcoded = the size of the arm)
		# Then the algorithm find the best poses amoung all of them for the robot_

		self.printMsg("> Computing the accessible poses")
		#accessible_poses2d = self.computeAccessiblePosesForVacuumCleaner()
		# if len(accessible_poses2d) == 0:
		# 	# todo - handler (rmb-ma)
		# 	self.printMsg("No accessible poses found.")
		# 	return
		# if self.handleInterrupt() >= 1:
		# 	return

		self.printMsg("> Computing the best accessible pose.")

		not_moved = True
		while not_moved:
			best_pose3d = self.computeBestPose()#ForRobot()#accessible_poses2d)
			if self.handleInterrupt() >= 1:
				return

			self.printMsg("> Moving to an accessible pose")
			self.printMsg(" Position ({}, {})".format(best_pose3d.position.x, best_pose3d.position.y))
			self.move_base_handler_.setParameters(
				goal_position=best_pose3d.position,
				goal_orientation=best_pose3d.orientation,
				header_frame_id='map',
				goal_angle_tolerance=0.2,
				goal_position_tolerance=0.10
			)
			self.move_base_handler_.executeBehavior()

			if self.handleInterrupt() >= 1:
				return

			if self.move_base_handler_.failed():
				self.printMsg('Pose not accessible')
				# remove the current position from the accessible ones
				return
				continue
			else:
				not_moved = False

		self.printMsg("> Todo. Remove the dirt")
		self.removeDirt()
		if self.handleInterrupt() >= 1:
			return

		self.printMsg("> Todo. Checkout the dirt")
		self.checkoutDirt()

if __name__ == '__main__':
	print "Test"
	dirt_remover = DirtRemovingBehavior(
		"DirtRemovingBehavior", None,
		move_base_service_str=None,
		map_accessibility_service_str=None,
		clean_pattern_str='/vacuum_cleaning_module_interface/clean_pattern')
	dirt_remover.removeDirt()

