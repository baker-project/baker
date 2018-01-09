#!/usr/bin/env python

import behavior_container

class TemplateBehavior(BehaviorContainer):

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		# After each command, def handleInterrupt has to be executed:
		# if handleInterrupt() != 0:
		#     return
		pass