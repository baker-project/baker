#!/usr/bin/env python

import rospy
import rospkg
import actionlib
import application_container
import map_handling_behavior
import dry_cleaning_behavior
import wet_cleaning_behavior
import database
import database_handler

from geometry_msgs.msg import Point32
import datetime

class WetCleaningApplication(application_container.ApplicationContainer):
	#========================================================================
	# Description:
	# Highest element in the hierarchy of the cleaning application
	#========================================================================

	# Dry cleaning routine, to be called from inside executeCustomBehavior()
	def processDryCleaning(self, rooms_dry_cleaning, is_overdue=False):

		if len(rooms_dry_cleaning) == 0:
			if not is_overdue:
				self.printMsg("There is no due room to be cleaned dry.")
			else:
				self.printMsg("There is no overdue room to be cleaned dry.")
			return

		# Get a sequence for all the rooms to be cleaned dry
		self.map_handler_.setParameters(
			self.database_handler_,
			rooms_dry_cleaning
		)
		self.map_handler_.executeBehavior()
		self.printMsg("Room Mapping (Dry): " + str(self.map_handler_.mapping_))
		for checkpoint in self.map_handler_.room_sequencing_data_.checkpoints:
			self.printMsg("Order: " + str(checkpoint.room_indices))

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Run Dry Cleaning Behavior
		self.dry_cleaner_.setParameters(
			database_handler=self.database_handler_,
			sequencing_result=self.map_handler_.room_sequencing_data_,
			mapping=self.map_handler_.mapping_,
			robot_radius=self.robot_radius_,
			coverage_radius=self.coverage_radius_,
			field_of_view=self.field_of_view_,
			field_of_view_origin=self.field_of_view_origin_,
			room_information_in_meter=self.database_handler_.getRoomInformationInMeter(rooms_dry_cleaning)
		)
		self.dry_cleaner_.executeBehavior()


	# Wet cleaning routine, to be called from inside executeCustomBehavior()
	def processWetCleaning(self, rooms_wet_cleaning, is_overdue=False):

		if len(rooms_wet_cleaning) == 0:
			if not is_overdue:
				self.printMsg("There is no due room to be cleaned wet.")
			else:
				self.printMsg("There is no overdue room to be cleaned wet.")
			return

		# Get a sequence for all the rooms to be cleaned wet
		self.map_handler_.setParameters(
			self.database_handler_,
			rooms_wet_cleaning
		)
		self.map_handler_.executeBehavior()
		self.printMsg("Room Mapping (Wet): " + str(self.map_handler_.mapping_))
		for checkpoint in self.map_handler_.room_sequencing_data_.checkpoints:
			self.printMsg("Order: " + str(checkpoint.room_indices))

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Run Wet Cleaning Behavior
		self.wet_cleaner_.setParameters(
			database_handler=self.database_handler_,
			room_information_in_meter=self.database_handler_.getRoomInformationInMeter(rooms_wet_cleaning),
			sequence_data=self.map_handler_.room_sequencing_data_,
			mapping=self.map_handler_.mapping_,
			robot_frame_id=self.robot_frame_id_,
			robot_radius=self.robot_radius_,
			coverage_radius=self.coverage_radius_,
			field_of_view=self.field_of_view_,
			field_of_view_origin=self.field_of_view_origin_,
			use_cleaning_device=self.use_cleaning_device_	# todo: hack: cleaning device can be turned off for trade fair show
		)
		self.wet_cleaner_.executeBehavior()

	# Implement application procedures of inherited classes here.
	def executeCustomBehavior(self, last_execution_date_override=None):

		# Initialize behaviors
		# ====================

		self.map_handler_ = map_handling_behavior.MapHandlingBehavior("MapHandlingBehavior", self.application_status_)
		self.dry_cleaner_ = dry_cleaning_behavior.DryCleaningBehavior("DryCleaningBehavior", self.application_status_)
		self.wet_cleaner_ = wet_cleaning_behavior.WetCleaningBehavior("WetCleaningBehavior", self.application_status_)

		# Load data from the robot device
		# ===============================

		# todo: read out these parameters
		if rospy.has_param('robot_frame'):
			self.robot_frame_id_ = rospy.get_param("robot_frame")
			self.printMsg("Imported parameter robot_frame = " + str(self.robot_frame_id_))
			# todo: write into database
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
		self.field_of_view_origin_ = Point32(x=0.0, y=0.0)	# todo: read from MIRA

		# todo: hack: cleaning device can be turned off for trade fair show
		self.use_cleaning_device_ = False
		if rospy.has_param('use_cleaning_device'):
			self.use_cleaning_device_ = rospy.get_param("use_cleaning_device")
			self.printMsg("Imported parameter use_cleaning_device = " + str(self.use_cleaning_device_))

		# Load database, initialize database handler
		# ==========================================

		# Initialize and load database
		#try:
		self.printMsg("Loading database from files...")
		rospack = rospkg.RosPack()
		print str(rospack.get_path('baker_wet_cleaning_application'))
		self.database_ = database.Database(extracted_file_path=str(rospack.get_path('baker_wet_cleaning_application') + "/resources"))
		self.database_.loadDatabase()
		#except:
		#	self.printMsg("Fatal: Loading of database failed! Stopping application.")
		#	exit(1)

		# Initialize database handler
		#try:
		self.database_handler_ = database_handler.DatabaseHandler(self.database_)
		#except:
		#	self.printMsg("Fatal: Initialization of database handler failed!")
		#	exit(1)

		shall_continue_old_cleaning = False
		days_delta = datetime.datetime.now() - self.database_.application_data_.last_execution_date_
		print "------------ CURRENT_DATE: " + str(datetime.datetime.now())
		print "------------ LAST_DATE: " + str(self.database_.application_data_.last_execution_date_)
		print "------------ DAYS_DELTA: " + str(days_delta) + " " + str(days_delta.days)
		if self.database_.application_data_.progress_[0] == 1:
			if days_delta.days == 0:
				shall_continue_old_cleaning = True
			else:
				self.printMsg("ERROR: Dates do not match! Shall the old progress be discarded?")
				# TODO: Programm needs to pause here. Then the user must be asked if the old cleaning state shall be overwritten.
		else:
			self.database_.application_data_.progress_ = [1, datetime.datetime.now()]
		self.database_handler_.applyChangesToDatabase()


		# Document start of the application
		# Also determine whether an old task is to be continued, independent of the current date
		# If datetime "last_execution_date_override" is not None, it will be set in the database.
		if last_execution_date_override is None:
			self.database_.application_data_.last_execution_date_ = datetime.datetime.now()
		else:
			self.database_.application_data_.last_execution_date_ = last_execution_date_override
		

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return


		# Find and sort all due rooms
		# ===========================

		# Find due rooms
		self.printMsg("Collecting due rooms...")
		self.database_handler_.due_rooms_ = []		# todo: verify whether this is correct here (otherwise the list of due rooms is corrupted with double and many-times occurring rooms)
		if self.database_handler_.noPlanningHappenedToday() and not shall_continue_old_cleaning:
			#try:
			# todo: check: it looks like the result of restoreDueRooms() could be erased in getAllDueRooms()
			self.database_handler_.restoreDueRooms()
			self.database_handler_.getAllDueRooms()
			print "len(self.database_.rooms_):", len(self.database_.rooms_), "\ndue rooms:"
			for room in self.database_handler_.due_rooms_:
				print room.room_name_
			#except:
			#	self.printMsg("Fatal: Collecting of the due rooms failed!")
			#	exit(1)
		else:
			#try:
			self.database_handler_.restoreDueRooms()
			#except:
			#	self.printMsg("Fatal: Restoring of the due rooms failed!")
			#	exit(1)

		# Sort the due rooms with respect to cleaning method
		self.printMsg("Sorting the found rooms with respect to cleaning method...")
		#try:
		rooms_dry_cleaning, rooms_wet_cleaning = self.database_handler_.sortRoomsList(self.database_handler_.due_rooms_)
		for room in rooms_dry_cleaning:
			self.printMsg(str(room.room_name_) + " ---> DRY")
		for room in rooms_wet_cleaning:
			self.printMsg(str(room.room_name_) + " ---> WET")

		#except:
		#	self.printMsg("Fatal: Sorting after the cleaning method failed!")
		#	exit(1)

		# Document completed due rooms planning in the database
		self.database_.application_data_.last_planning_date_[0] = datetime.datetime.now()
		self.database_handler_.applyChangesToDatabase()

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Dry cleaning of the due rooms
		# =============================
		self.processDryCleaning(rooms_dry_cleaning, is_overdue=False)


		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return


		# Wet cleaning of the due rooms
		# =============================
		self.processWetCleaning(rooms_wet_cleaning, is_overdue=False)
		
		#self.printMsg("self.map_handler_.room_sequencing_data_.checkpoints=" + str(self.map_handler_.room_sequencing_data_.checkpoints))
		
		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Find and sort all overdue rooms
		# ===============================

		# Find overdue rooms
		self.printMsg("Collecting overdue rooms...")
		#try:
		self.database_handler_.getAllOverdueRooms()
		#except:
		#	self.printMsg("Fatal: Collecting of the over rooms failed!")
		#	exit(1)

		# Sort the overdue rooms after cleaning method
		self.printMsg("Sorting the found rooms after cleaning method...")
		#try:
		rooms_dry_cleaning, rooms_wet_cleaning = self.database_handler_.sortRoomsList(self.database_handler_.overdue_rooms_)
		#except:
		#	self.printMsg("Fatal: Sorting after the cleaning method failed!")
		#	exit(1)

		# Document completed due rooms planning in the database
		self.database_.application_data_.last_planning_date_[1] = datetime.datetime.now()
		self.database_handler_.applyChangesToDatabase()

		# Dry cleaning of the overdue rooms
		# =================================

		self.processDryCleaning(rooms_dry_cleaning, True)

		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Wet cleaning of the overdue rooms
		# =================================

		self.processWetCleaning(rooms_wet_cleaning, True)
		
		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# COMPLETE APPLICATION
		# ====================

		self.printMsg("Cleaning completed. Overwriting database...")
		#try:
		self.database_.application_data_.progress_ = [0, datetime.datetime.now()]
		self.database_handler_.cleaningFinished()
		#except:
		#	self.printMsg("Fatal: Database overwriting failed!")
		#	exit(1)

	# Abstract method that contains the procedure to be done immediately after the application is paused.
	def prePauseProcedure(self):
		self.printMsg("Application paused.")
		# save current data if necessary
		# undo or check whether everything has been undone
		self.returnToRobotStandardState()

	# Abstract method that contains the procedure to be done immediately after the pause has ended
	def postPauseProcedure(self):
		self.printMsg("Application continued.")

	# Abstract method that contains the procedure to be done immediately after the application is cancelled.
	def cancelProcedure(self):
		self.printMsg("Application cancelled.")
		# save current data if necessary
		# undo or check whether everything has been undone
		self.returnToRobotStandardState()

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
		self.database_.saveRoomDatabase()
		self.database_.saveGlobalApplicationData()
		# undo or check whether everything has been undone


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
