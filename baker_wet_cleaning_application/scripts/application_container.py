#!/usr/bin/env python

import roslib
roslib.load_manifest('baker_wet_cleaning_application')

import rospy
import threading
from abc import ABCMeta, abstractmethod
import std_msgs
from cob_srvs.srv import SetInt, SetIntResponse


class ApplicationContainer:

	# ========================================================================
	# Description:
	# Abstract class which contains the overall application
	# ========================================================================

	__metaclass__ = ABCMeta
	# Arbitrary application name. Only used for debug.
	application_name_ = "<Unnamed>"

	# Status of the application. 0=OK, 1=Paused, 2=Cancelled, 3=Terminate application server
	# Starts with 2=Cancelled and waits for an action call to start the application
	# using a vector because a single int number cannot be passed by reference, 
	# but this apparently works to automatically get the changed number also into the client behaviors
	STATUS = {'IS_RUNNING': 0, 'IS_PAUSED': 1, 'IS_CANCELLED': 2, 'IS_FINISHED': 3}
	application_status_ = [STATUS['IS_CANCELLED']]

	def __init__(self, application_name, interrupt_action_name):
		self.application_name_ = application_name
		# Initialize the interruption action server
		self.interrupt_action_name_ = interrupt_action_name
		self.interrupt_server_ = rospy.Service(interrupt_action_name, SetInt, self.interruptCallback)

		threading.Thread(target=self.publishApplicationStatus).start()

		# needs to be set to true after a pause for continuing the application
		self.application_resumed_after_pause = False

	def publishApplicationStatus(self):
		application_status_pub = rospy.Publisher(str(self.application_name_) + '_status', std_msgs.msg.Int32, queue_size=1)
		rate = rospy.Rate(5)  # 5hz
		while not rospy.is_shutdown():
			application_status_pub.publish(self.getStatus())
			rate.sleep()

	# Method for printing messages.
	def printMsg(self, text):
		print "[Application '" + str(self.application_name_) + "']: " + str(text)

	# Callback function for interrupt
	def interruptCallback(self, req):
		self.setStatus(req.data)
		self.printMsg('Changed status to {}'.format(self.getStatus()))
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
		if self.getStatus() == self.STATUS['IS_PAUSED']:
			self.prePauseProcedure()
			while self.getStatus() == self.STATUS['IS_PAUSED']:
				pass
			if self.getStatus() == self.STATUS['IS_RUNNING']:
				self.application_resumed_after_pause = True
				self.postPauseProcedure()

		elif self.getStatus() == self.STATUS['IS_CANCELLED']:
			self.cancelProcedure()

		elif self.getStatus() == self.STATUS['IS_FINISHED']:
			self.cancelProcedure()

		print(self.getStatus())
		return self.getStatus()

	# Implement application procedures of inherited classes here.
	@abstractmethod
	def executeCustomBehavior(self):
		# if handleInterrupt() >= 1:
		# 	return
		# has to be inserted after each command
		pass

	# Call this method to execute the application.
	def executeApplication(self):
		self.printMsg("Application server started.")
		rate = rospy.Rate(10)  # 10hz
		while not rospy.is_shutdown():
			if self.getStatus() == self.STATUS['IS_RUNNING']:
				self.printMsg("Application started.")
				self.executeCustomBehavior()
				if not self.application_resumed_after_pause:
					if self.getStatus() == self.STATUS['IS_RUNNING']:
						self.setStatus(self.STATUS['IS_CANCELLED'])
					self.printMsg("Application completed with code {}".format(self.getStatus()))

			elif self.getStatus() == self.STATUS['IS_FINISHED']:
				break
			
			rate.sleep()

	def setStatus(self, status):
		self.application_status_[0] = status

	def getStatus(self):
		return self.application_status_[0]

