#!/usr/bin/env python

import rospy
import tf
from abc import ABCMeta, abstractmethod
from threading import Lock
from application_container import ApplicationContainer

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
	# ========================================================================
	# Description:
	# Abstract class which contains one specific behavior of the application
	# ========================================================================

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

	# This function is ugly but is necessary to stop a specific behavior
	# without stopping the whole application
	# It's used in DryCleaningBehavior to stop the path follower once a dirt or
	# a trashcan is detected
	# This function is used with 'setInterruptVar'
	def interruptExecution(self):
		self.mutex_.acquire()
		self.interrupt_var_ = [ApplicationContainer.STATUS['IS_CANCELLED']]
		self.mutex_.release()

	def setInterruptVar(self, interrupt_var):
		self.interrupt_var_ = interrupt_var

	# Method that returns the current interruption value [True/False]
	def executionInterrupted(self):
		self.mutex_.acquire()
		is_interrupted = self.interrupt_var_[0] != ApplicationContainer.STATUS['IS_RUNNING'] \
			and self.interrupt_var_[0] != ApplicationContainer.STATUS['IS_PAUSED']
		self.mutex_.release()
		return is_interrupted

	def executionPaused(self):
		self.mutex_.acquire()
		is_paused = self.interrupt_var_[0] == ApplicationContainer.STATUS['IS_PAUSED']
		self.mutex_.release()
		return is_paused

	# Method that handles interruptions (ASSUMING: False=OK, True=INTERRUPT)
	def handleInterrupt(self):
		if self.executionInterrupted():
			self.printMsg("Interrupted")
			self.returnToRobotStandardState()
			self.printMsg("Execution interrupted with code " + str(self.behavior_status_))
		return self.interrupt_var_[0]

	def failed(self):
		return self.state_ != 3

	def computeNewGoalFromPausedResult(self, prev_action_goal, result):
		return prev_action_goal

	# Method for running an action server, shall only be called from def executeCustomBehavior
	# Definition of SimpleGoalState: 0 = PENDING, 1 = ACTIVE, 3 = DONE
	def runAction(self, action_client, action_goal):

		self.is_finished = False
		self.printMsg("Waiting for action " + str(action_client.action_client.ns) + " to become available...")
		action_client.wait_for_server()

		# loop --> ask for action finished and sleep for one second
		# in loop check for interrupt --> if necessary stop action with self.executionInterrupted() == True and wait until action stopped
		while not self.is_finished:
			resumed_after_pause = False
			# action client --> call external functionality but do not wait for finishing
			self.printMsg("Sending goal...")
			action_client.send_goal(action_goal)

			while action_client.get_state() < 3:
				if self.executionPaused():
					action_client.cancel_goal()
					action_client.wait_for_result()
					result = action_client.get_result()

					while self.executionPaused() and not rospy.is_shutdown():
						rospy.sleep(self.sleep_time_)

					if self.interrupt_var_[0] == ApplicationContainer.STATUS['IS_RUNNING'] and not rospy.is_shutdown():
						action_goal = self.computeNewGoalFromPausedResult(prev_action_goal=action_goal, result=result)
						resumed_after_pause = True

				if self.executionInterrupted() or rospy.is_shutdown():
					action_client.cancel_goal()
					while action_client.get_state() < 3 and not rospy.is_shutdown():
						rospy.sleep(self.sleep_time_)
					action_client.wait_for_result()
					self.is_finished = True

					return {'interrupt_var': self.handleInterrupt(), 'result': action_client.get_result()}
				rospy.sleep(self.sleep_time_)

			if resumed_after_pause:
				continue

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
