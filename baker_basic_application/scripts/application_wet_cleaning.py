#!/usr/bin/env python

import rospy
import actionlib
import application_container
import map_handling_behavior

class WetCleaningApplication(application_container.ApplicationContainer):

	#========================================================================
	# Serivces to be used:
	# map_receiving_service_str:
	#       '/baker/get_map_image'
	# map_segmentation_service_str:
	#       '/room_segmentation/room_segmentation_server'
	# room_sequencing_service_str = 
	#       '/room_sequence_planning/room_sequence_planning_server'
	#========================================================================

	# Implement application procedures of inherited classes here.
	def executeCustomBehavior(self):
		# Receive map, segment, get sequence, convert to openCV
		self.map_handler = map_handling_behavior.MapHandlingBehavior(
			self.application_status,
			'/baker/get_map_image',
			'/room_segmentation/room_segmentation_server',
			'/room_sequence_planning/room_sequence_planning_server'
			)
		self.map_handler.executeCustomBehavior()
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