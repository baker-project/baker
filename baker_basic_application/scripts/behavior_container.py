#!/usr/bin/env python

import roslib
roslib.load_manifest('baker_office_cleaning_application')
import actionlib
import rospy
import sys
import time
from abc import ABCMeta, abstractmethod


class BehaviorContainer:
	# Make an abstract class out of it
	__metaclass__ = ABCMeta
	# Behaviors which this behavior contains
	sub_behaviors = []
	# Status of the behavior. 0=OK, 1=Erroneous
	behavior_status = 0
	# Sleeping time in seconds
	sleep_time = 1

	# Constructor
	def __init__(self, interrupt_var):
		# Get the pointer to the interrupt variable of the application container
		self.interrupt_var = interrupt_var

	# Method that returns the current interruption value [True/False]
	def executionInterrupted(self):
		return (self.interrupt_var[0] != 0)

	# Method that handles interruptions (ASSUMING: False=OK, True=INTERRUPT)
	def handleInterrupt(self):
		if (self.interrupt_var[0] != 0):
			print "Behavior: Interrupted"
			self.returnToRobotStandardState()
			print "Behavior: Execution interrupted with code " + str(behavior_status)
		return self.interrupt_var[0]

	# Method for running an action server
	def runAction(self, action_client, action_goal):
		# action client --> call external functionality but do not wait for finishing
		action_client.wait_for_server()
		action_client.send_goal(action_goal)
		action_result = action_client.get_result()
		# loop --> ask for action finished and sleep for one second
		# in loop check for interrupt --> if necessary stop action with self.executionInterrupted() == True and wait until action stopped
		while (action_client.get_state() != action_client.SUCCEEDED):
			if (executionInterrupted() == True):
				action_client.cancel_goal()
				return handle_interrupt()
			time.sleep(sleep_time)
		return action_result


	# Method for returning to the standard pose of the robot
	@abstractmethod
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass
    
	# Implement behavior procedures of inherited classes here.
	@abstractmethod
	def executeCustomBehavior(self):
		# After each command, def handleInterrupt has to be executed:
		# if handleInterrupt() != 0:
		#     return
		pass

	# Call this function to execute the behavior.
	def executeBehavior(self):
		print "Behavior: Executing behavior..."
		executeCustomBehavior()
		print "Behavior: Execution ended with code " + str(behavior_status)
