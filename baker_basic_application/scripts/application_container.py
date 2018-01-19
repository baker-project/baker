#!/usr/bin/env python

import roslib
roslib.load_manifest('baker_basic_application')

import actionlib
import rospy
import sys
from abc import ABCMeta, abstractmethod

from baker_basic_application.msg import InterruptActionAction
from baker_basic_application.msg import InterruptActionGoal
from baker_basic_application.msg import InterruptActionResult

class ApplicationContainer:
	__metaclass__ = ABCMeta
	# Arbitrary application name. Only used for debug.
	application_name = "<Unnamed>"
	# Status of the application. 0=OK, 1=Paused, 2=Cancelled
	# using a vector because a single int number cannot be passed by reference, 
	# but this apparently works to automatically get the changed number also into the client behaviors
	application_status = [0]


	# Method for printing messages.
	def printMsg(self, text_):
		print "[Application '" + str(self.application_name) + "']: " + str(text_)

	# Constructor
	def __init__(self, interrupt_action_name_):
		# Initialize the interruption action server
		self.interrupt_action_name = interrupt_action_name_
		self.interrupt_server = actionlib.SimpleActionServer(interrupt_action_name_, InterruptActionAction, execute_cb=self.interruptCallback, auto_start=False)
		self.interrupt_server.start()

	# Callback function for interrupt
	def interruptCallback(self, goal):
		self.application_status[0] = goal.new_interrupt_state
		result_ = InterruptActionResult()
		result_.interrupt_state = self.application_status[0]
		self.interrupt_server.set_succeeded(result_)

	# Abstract method that contains the procedure to be done immediately after the application is paused.
	@abstractmethod
	def prePauseProcedure(self):
		self.printMsg("Application paused.")
		# save current data if necessary
		# undo or check whether everything has been undone
		self.returnToRobotStandardState()

	# Abstract method that contains the procedure to be done immediately after the pause has ended
	@abstractmethod
	def postPauseProcedure(self):
		self.printMsg("Application continued.")


	# Abstract method that contains the procedure to be done immediately after the application is cancelled.
	@abstractmethod
	def cancelProcedure(self):
		self.printMsg("Application cancelled.")
		# save current data if necessary
		# undo or check whether everything has been undone
		self.returnToRobotStandardState()

	# Method for returning to the standard pose of the robot
	@abstractmethod   
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass
		
	# Handle interruption, if one occurs
	def handleInterrupt(self):
		self.printMsg("Current status is " + str(self.application_status[0]))
		if (self.application_status[0] == 1):
			self.prePauseProcedure()
			while (self.application_status[0] == 1):
				pass
			self.postPauseProcedure()
		elif (self.application_status[0] == 2):
			self.cancelProcedure()
		return self.application_status[0]

	# Implement application procedures of inherited classes here.
	@abstractmethod
	def executeCustomBehavior(self):
		# After each command,
		# if handleInterrupt() == 2:
		# 	return
		# has to be inserted.
		pass

	# Call this method to execute the application.
	def executeApplication(self):
		self.printMsg("Started.")
		if self.handleInterrupt() != 0:
			pass
		self.executeCustomBehavior()
		self.printMsg("Completed with code " + str(self.application_status[0]))
