#!/usr/bin/env python

import application_container

class TemplateApplication(ApplicationContainer):

	# Implement all procedures to be done once at the beginning here.
	def initialize(self):
		pass

	# Implement application procedures of inherited classes here.
	def executeCustomBehavior(self):
		# After each command,
		# if self.handleInterrupt() == 2:
		#     return
		# has to be inserted.
		pass

	# Abstract method that contains the procedure to be done immediately after the application is paused.
	def pauseProcedure(self):
		print "Application: Application paused."
		# save current data if necessary
		# undo or check whether everything has been undone
		self.returnToRobotStandardState()

	# Abstract method that contains the procedure to be done immediately after the application is cancelled.
	def cancelProcedure(self):
		print "Application: Application cancelled."
		# save current data if necessary
		# undo or check whether everything has been undone
		self.returnToRobotStandardState()


	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		# undo or check whether everything has been undone
		pass