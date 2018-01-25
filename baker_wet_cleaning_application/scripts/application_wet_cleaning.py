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
		self.map_handler_ = map_handling_behavior.MapHandlingBehavior("MapHandlingBehavior", self.application_status_)
		self.map_handler_.setParameters()
		self.map_handler_.executeBehavior()
		
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return
		
		self.printMsg("self.map_handler_.room_sequencing_data.checkpoints=")
		print self.map_handler_.room_sequencing_data.checkpoints
		
		# Move to segments, Compute exploration path, Travel through it, repeat
		self.movement_handler_ = movement_handling_behavior.MovementHandlingBehavior("MovementHandlingBehavior", self.application_status_)
		self.movement_handler_.setParameters(
			self.map_handler_.map_data,
			self.map_handler_.segmentation_data,
			self.map_handler_.room_sequencing_data,
		)
		self.movement_handler_.executeBehavior()
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return

	# Abstract method that contains the procedure to be done immediately after the application is paused.
	def prePauseProcedure(self):
		print "Application: Application paused."
		# save current data if necessary
		# undo or check whether everything has been undone
		self.returnToRobotStandardState()

	# Abstract method that contains the procedure to be done immediately after the pause has ended
	def postPauseProcedure(self):
		print "Application: Application continued."

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
		app = WetCleaningApplication("application_wet_cleaning", "interrupt_application_wet_cleaning")
		# Execute application
		app.executeApplication()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
	rospy.signal_shutdown("Finished")