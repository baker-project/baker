#!/usr/bin/env python

import roslib
roslib.load_manifest('baker_wet_cleaning_application')
import rospy
import tf
from abc import ABCMeta, abstractmethod
from threading import Lock

_tl=None
_tl_creation_lock = Lock()


def get_transform_listener():
	global _tl
	with _tl_creation_lock:
		if _tl is None:
			_tl = tf.TransformListener(True, rospy.Duration(40.0))
		return _tl
###########################################################################

class BehaviorContainer:
	#========================================================================
	# Description:
	# Abstract class which contains one specific behavior of the application
	#========================================================================

	__metaclass__ = ABCMeta
	# Arbitrary behavior name. Only used for debug.
	behavior_name_ = "<Unnamed>"
	# Status of the behavior. 0=OK, 1=Cancelled, 2=Erroneous (use methods isOk, isCancelled, isErroneous)
	behavior_status_ = 0
	# Sleeping time in seconds
	sleep_time_ = 1

	# Constructor
	def __init__(self, behavior_name, interrupt_var):
		self.behavior_name_ = behavior_name
		# Get the pointer to the interrupt variable of the application container
		self.interrupt_var_ = interrupt_var
		self.mutex_ = Lock()
		self.is_finished = False
		self.state_ = None

	# Method for printing messages.
	def printMsg(self, text):
		print "[Behavior '" + str(self.behavior_name_) + "']: " + str(text)

	def interruptExecution(self):
		self.mutex_.acquire()
		self.interrupt_var_ = [1]
		self.mutex_.release()

	def setInterruptVar(self, interrupt_var):
		self.interrupt_var_ = interrupt_var

	# Method that returns the current interruption value [True/False]
	def executionInterrupted(self):
		self.mutex_.acquire()
		is_interrupted = self.interrupt_var_[0] != 0
		self.mutex_.release()
		return is_interrupted

	# Method that handles interruptions (ASSUMING: False=OK, True=INTERRUPT)
	def handleInterrupt(self):
		if self.executionInterrupted():
			self.printMsg("Interrupted")
			self.returnToRobotStandardState()
			self.printMsg("Execution interrupted with code " + str(self.behavior_status_))
		return self.interrupt_var_[0]

	def failed(self):
		return self.state_ != 3

	# Method for running an action server, shall only be called from def executeCustomBehavior
	def runAction(self, action_client, action_goal):
		self.is_finished = False
		# action client --> call external functionality but do not wait for finishing
		self.printMsg("Waiting for action " + str(action_client.action_client.ns) + " to become available...")
		action_client.wait_for_server()
		self.printMsg("Sending goal...")
		action_client.send_goal(action_goal)
		# loop --> ask for action finished and sleep for one second
		# in loop check for interrupt --> if necessary stop action with self.executionInterrupted() == True and wait until action stopped
		# Definition of SimpleGoalState: 0 = PENDING, 1 = ACTIVE, 3 = DONE
		while action_client.get_state() < 3:
			#self.printMsg("action_client.get_state()=" + str(action_client.get_state()))
			if self.executionInterrupted() or rospy.is_shutdown():
				action_client.cancel_goal()
				while action_client.get_state() < 3 and not rospy.is_shutdown():
					pass
				action_client.wait_for_result()
				self.is_finished = True
				return {'interrupt_var': self.handleInterrupt(), 'result': action_client.get_result()}
			rospy.sleep(self.sleep_time_)
		if action_client.get_state() == 3:
			self.printMsg("Action successfully processed.")
		else:
			self.printMsg("Action failed.")

		action_client.wait_for_result()
		self.is_finished = True
		self.state_ = action_client.get_state()
		return {'interrupt_var': self.interrupt_var_[0], 'result': action_client.get_result()}

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

	def isOk(self):
		return self.behavior_status_ == 0

	def isCancelled(self):
		return self.behavior_status_ == 1

	def isErroneous(self):
		return self.behavior_status_ == 2
