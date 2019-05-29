#!/usr/bin/env python

from scripts.tests import behavior_container


class TrolleyMovementBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Class which contains the behavior of moving the trolley
	# to another position
	#========================================================================
		
	# Method for setting parameters for the behavior
	def setParameters(self, database_handler):
		self.database_handler_ = database_handler

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		pass
