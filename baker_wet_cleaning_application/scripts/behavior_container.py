#!/usr/bin/env python

import roslib
roslib.load_manifest('baker_wet_cleaning_application')
import actionlib
import rospy
import sys
import time
from abc import ABCMeta, abstractmethod


class BehaviorContainer:
	__metaclass__ = ABCMeta
	# Arbitrary behavior name. Only used for debug.
	behavior_name_ = "<Unnamed>"
	# Status of the behavior. 0=OK, 1=Cancelled, 2=Erroneous
	behavior_status_ = 0
	# Sleeping time in seconds
	sleep_time_ = 1


# Method for printing messages.
	def printMsg(self, text):
		print "[Behavior '" + str(self.behavior_name_) + "']: " + str(text)

	# Constructor
	def __init__(self, behavior_name, interrupt_var):
		self.behavior_name_ = behavior_name
		# Get the pointer to the interrupt variable of the application container
		self.interrupt_var_ = interrupt_var

		# Method that returns the current interruption value [True/False]
	def executionInterrupted(self):
		return (self.interrupt_var_[0] != 0)

	# Method that handles interruptions (ASSUMING: False=OK, True=INTERRUPT)
	def handleInterrupt(self):
		if self.executionInterrupted() == True:
			self.printMsg("Interrupted")
			self.returnToRobotStandardState()
			self.printMsg("Execution interrupted with code " + str(self.behavior_status_))
		return self.interrupt_var_[0]

	# Method for running an action server, shall only be called from def executeCustomBehavior
	def runAction(self, action_client, action_goal):
		# action client --> call external functionality but do not wait for finishing
		self.printMsg("Waiting for action " + str(action_client.action_client.ns) + " to become available...")
		action_client.wait_for_server()
		self.printMsg("Sending goal...")
		action_client.send_goal(action_goal)
		# loop --> ask for action finished and sleep for one second
		# in loop check for interrupt --> if necessary stop action with self.executionInterrupted() == True and wait until action stopped
		# Definition of SimpleGoalState: 0 = PENDING, 1 = ACTIVE, 3 = DONE (don't ask why DONE is 3)
		while (action_client.get_state() != 3):
			# print str(action_client.get_state())
			if (self.executionInterrupted() == True):
				action_client.cancel_goal()
				return self.handleInterrupt()
			rospy.sleep(self.sleep_time_)
		action_result = action_client.get_result()
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
		self.printMsg("Executing behavior...")
		self.executeCustomBehavior()
		self.printMsg("Execution ended with code " + str(self.behavior_status_))
