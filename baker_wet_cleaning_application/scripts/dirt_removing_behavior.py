#!/usr/bin/env python

import rospy
import behavior_container
import database_handler
from move_base_behavior import MoveBaseBehavior
from geometry_msgs.msg import Quaternion


class DirtRemovingBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Class which contains the behavior for removing dirt spots
	# > Go to the dirt location
	# > Clean
	#========================================================================

	def __init__(self, behavior_name, interrupt_var, move_base_service_str):
		super(DirtRemovingBehavior, self).__init__(behavior_name, interrupt_var)
		self.move_base_handler_ = MoveBaseBehavior("MoveBaseBehavior", self.interrupt_var_, move_base_service_str)
		self.dirt_position_ = None

	# Method for setting parameters for the behavior
	def setParameters(self, dirt_position):
		self.dirt_position_ = dirt_position

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		assert(self.dirt_position_ is not None)

		self.printMsg("Removing dirt located on ({}, {})".format(self.dirt_position_.x, self.dirt_position_.y))

		self.printMsg("> Moving to the dirt location")
		self.move_base_handler_.setParameters(
			goal_position=self.dirt_position_,
			goal_orientation=Quaternion(x=0., y=0., z=0., w=1.),
			header_frame_id='base_link'
		)
		self.move_base_handler_.executeBehavior()

		self.printMsg("> Todo. Remove the dirt")
