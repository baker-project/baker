#!/usr/bin/env python

import behavior_container

class TemplateBehavior(behavior_container.BehaviorContainer):

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass

	# Method for setting parameters for the behavior
	def setParameters(self):
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		# After each command, def handleInterrupt has to be executed:
		# if self.handleInterrupt() == 2:
		#     return
		pass


'''
How to call the implemented behavior:

xyz = behavior_template.TemplateBehavior(<interrupt_var>)
xyz.behavior_name = <behavior_name>
xyz.setParameters(<parameters>)

...

xyz.executeBehavior()
'''