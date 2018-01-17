#!/usr/bin/env python

import rospy
import actionlib
import application_container
import map_handling_behavior
import movement_handling_behavior

class WetCleaningApplication(application_container.ApplicationContainer):

	# Implement application procedures of inherited classes here.
	def executeCustomBehavior(self):
		# Receive map, segment, get sequence, extract maps
		self.map_handler = map_handling_behavior.MapHandlingBehavior(self.application_status)
		self.map_handler.behavior_name = "Map handling"
		self.map_handler.setParameters()
		self.map_handler.executeBehavior()
        # Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		# Move to segments, Compute exploration path, Travel through it, repeat
		self.movement_handler = movement_handling_behavior.MovementHandlingBehavior(self.application_status)
		self.movement_handler.behavior_name = "Movement handling"
		self.movement_handler.setParameters(
			self.map_handler.map_data,
			self.map_handler.room_sequencing_data,
			self.map_handler.room_extraction_data
		)
		self.movement_handler.executeBehavior()
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return

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



if __name__ == '__main__':
	try:
		# Initialize node
		rospy.init_node('application_wet_cleaning')
		# Initialize application
		app = WetCleaningApplication("interrupt_application_wet_cleaning")
		# Execute application
		app.executeApplication()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"