#!/usr/bin/env python

import behavior_container
import move_base_behavior

class MovementHandlingBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, map_data_, extraction_data_):
		self.interrupt_var = interrupt_var_
		self.map_data = map_data_
		self.extraction_data = extraction_data_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		# After each command, def handleInterrupt has to be executed:
		# if self.handleInterrupt() == 2:
		#     return
		pass