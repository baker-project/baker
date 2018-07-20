#!/usr/bin/env python

import roslib
roslib.load_manifest('baker_wet_cleaning_application')

import actionlib
import rospy
import sys
import threading
from abc import ABCMeta, abstractmethod
import std_msgs
from cob_srvs.srv import SetInt, SetIntResponse

from baker_wet_cleaning_application.msg import InterruptActionAction
from baker_wet_cleaning_application.msg import InterruptActionGoal
from baker_wet_cleaning_application.msg import InterruptActionResult

class ApplicationContainer:

	#========================================================================
	# Description:
	# Abstract class which contains the overall application
	#========================================================================

	__metaclass__ = ABCMeta
	# Arbitrary application name. Only used for debug.
	application_name_ = "<Unnamed>"
	# Status of the application. 0=OK, 1=Paused, 2=Cancelled, 3=Terminate application server
	# Starts with 1=Paused and waits for an action call to start the application
	# using a vector because a single int number cannot be passed by reference, 
	# but this apparently works to automatically get the changed number also into the client behaviors
	application_status_ = [1]

	def publishApplicationStatus(self):
		self.application_status_pub_ = rospy.Publisher(str(self.application_name_) + '_status', std_msgs.msg.Int32, queue_size=1)
		rate = rospy.Rate(5) # 5hz
		while (not rospy.is_shutdown()):
			self.application_status_pub_.publish(self.application_status_[0])
			rate.sleep()

	# Method for printing messages.
	def printMsg(self, text):
		print "[Application '" + str(self.application_name_) + "']: " + str(text)

	# Constructor
	def __init__(self, application_name, interrupt_action_name):
		self.application_name_ = application_name
		# Initialize the interruption action server
		self.interrupt_action_name_ = interrupt_action_name
		#self.interrupt_server_ = actionlib.SimpleActionServer(interrupt_action_name, InterruptActionAction, execute_cb=self.interruptCallback, auto_start=False)
		#self.interrupt_server_.start()
		self.interrupt_server_ = rospy.Service(interrupt_action_name, SetInt, self.interruptCallback)
		
		thread = threading.Thread(target = self.publishApplicationStatus)
		thread.start()

	# Callback function for interrupt
	def interruptCallback(self, req):
		self.application_status_[0] = req.data
		print "Changed self.application_status_[0] =", self.application_status_[0]
		res = SetIntResponse()
		res.success = True
		return res

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
		self.printMsg("Current status is " + str(self.application_status_[0]))
		if (self.application_status_[0] == 1):
			self.prePauseProcedure()
			while (self.application_status_[0] == 1):
				pass
			if self.application_status_[0] == 0:
				self.postPauseProcedure()
		elif (self.application_status_[0] == 2):
			self.cancelProcedure()
		elif (self.application_status_[0] == 3):
			self.cancelProcedure()
		return self.application_status_[0]

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
		self.printMsg("Application server started.")
		while not rospy.is_shutdown():
			#if self.handleInterrupt() != 0:
			#	pass
			if self.application_status_[0] == 0:
				self.printMsg("Application started.")
				self.executeCustomBehavior()
				if self.application_status_[0] == 0:
					self.application_status_[0] = 1		# set back to 1=Paused after successful, uninterrupted execution to avoid automatic restart
				self.printMsg("Application completed with code " + str(self.application_status_[0]))
			elif self.application_status_[0] == 3:
				break