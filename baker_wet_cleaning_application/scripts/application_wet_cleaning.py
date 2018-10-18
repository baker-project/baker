#!/usr/bin/env python

import rospy
import actionlib
import application_container
import map_handling_behavior
import movement_handling_behavior

from geometry_msgs.msg import Point32

class WetCleaningApplication(application_container.ApplicationContainer):

	# Implement application procedures of inherited classes here.
	def executeCustomBehavior(self):
		
		# todo: read out these parameters
		if rospy.has_param('robot_frame'):
			self.robot_frame_id_ = rospy.get_param("robot_frame")
			self.printMsg("Imported parameter robot_frame = " + str(self.robot_frame_id_))
		if rospy.has_param('robot_radius'):
			self.robot_radius_ = rospy.get_param("robot_radius")
			self.printMsg("Imported parameter robot_radius = " + str(self.robot_radius_))
		if rospy.has_param('coverage_radius'):
			self.coverage_radius_ = rospy.get_param("coverage_radius")
			self.printMsg("Imported parameter robot_radius = " + str(self.coverage_radius_))
		# todo: get field_of_view

		#self.robot_frame_id_ = 'base_link'
		#self.robot_radius_ = 0.2875  #0.325	# todo: read from MIRA
		#self.coverage_radius_ = 0.233655  #0.25	# todo: read from MIRA
		self.field_of_view_ = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364),
							   Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)]	# todo: read from MIRA

		# todo: hack: cleaning device can be turned off for trade fair show
		self.use_cleaning_device_ = False
		if rospy.has_param('use_cleaning_device'):
			self.use_cleaning_device_ = rospy.get_param("use_cleaning_device")
			self.printMsg("Imported parameter use_cleaning_device = " + str(self.use_cleaning_device_))
		
		
		# Receive map, segment, get sequence, extract maps
		self.map_handler_ = map_handling_behavior.MapHandlingBehavior("MapHandlingBehavior", self.application_status_)
		self.map_handler_.setParameters(
			self.robot_radius_
		)
		self.map_handler_.executeBehavior()
		
		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return
		
		#self.printMsg("self.map_handler_.room_sequencing_data_.checkpoints=" + str(self.map_handler_.room_sequencing_data_.checkpoints))
		
		# Move to segments, Compute exploration path, Travel through it, repeat
		self.movement_handler_ = movement_handling_behavior.MovementHandlingBehavior("MovementHandlingBehavior", self.application_status_)
		self.movement_handler_.setParameters(
			self.map_handler_.map_data_,
			self.map_handler_.segmentation_data_,
			self.map_handler_.room_sequencing_data_,
			self.robot_frame_id_,
			self.robot_radius_,
			self.coverage_radius_,
			self.field_of_view_,
			self.use_cleaning_device_	# todo: hack: cleaning device can be turned off for trade fair show
		)
		self.movement_handler_.executeBehavior()
		# Interruption opportunity
		if self.handleInterrupt() >= 1:
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
		app = WetCleaningApplication("application_wet_cleaning", "set_application_status_application_wet_cleaning")
		# Execute application
		app.executeApplication()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
	rospy.signal_shutdown("Finished")