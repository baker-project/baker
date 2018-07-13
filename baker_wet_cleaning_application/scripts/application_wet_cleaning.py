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
	def processDryCleaning(self, rooms_dry_cleaning, is_overdue):

		if (rooms_dry_cleaning != []):

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
			if self.handleInterrupt() == 2:
				return

			# Run Dry Cleaning Behavior
			self.dry_cleaner_.setParameters(
				self.database_handler_,
				self.map_handler_.room_sequencing_data_,
				self.map_handler_.mapping_
			)
			self.dry_cleaner_.executeBehavior()
		
		else:
			if (is_overdue == False):
				self.printMsg("There is no due room to be cleaned dry.")
			else:
				self.printMsg("There is no overdue room to be cleaned dry.")



	# Wet cleaning routine, to be called from inside executeCustomBehavior()
	def processWetCleaning(self, rooms_wet_cleaning, is_overdue):

		if (rooms_wet_cleaning != []):

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
			if self.handleInterrupt() == 2:
				return

			# Run Wet Cleaning Behavior
			self.wet_cleaner_.setParameters(
				self.database_handler_,
				self.database_handler_.database_.global_map_data_.map_image_segmented_,
				self.database_handler_.getRoomInformationInMeter(rooms_wet_cleaning),
				self.map_handler_.room_sequencing_data_,
				self.map_handler_.mapping_,
				self.robot_frame_id_,
				self.robot_radius_,
				self.coverage_radius_,
				self.field_of_view_
			)
			self.wet_cleaner_.executeBehavior()

		else:
			if (is_overdue == False):
				self.printMsg("There is no due room to be cleaned wet.")
			else:
				self.printMsg("There is no overdue room to be cleaned wet.")




	# Implement application procedures of inherited classes here.
	def executeCustomBehavior(self):


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
		if rospy.has_param('robot_radius'):
			self.robot_radius_ = rospy.get_param("robot_radius")
			self.printMsg("Imported parameter robot_radius = " + str(self.robot_radius_))
		if rospy.has_param('coverage_radius'):
			self.coverage_radius_ = rospy.get_param("coverage_radius")
			self.printMsg("Imported parameter robot_radius = " + str(self.coverage_radius_))
		# todo: get field_of_view

		self.field_of_view_ = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364),
							   Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)]	# todo: read from MIRA

		

		# Load database, initialize database handler
		# ==========================================

		# Initialize and load database
		#try:
		self.printMsg("Loading database from files...")
		rospack = rospkg.RosPack()
		print str(rospack.get_path('baker_wet_cleaning_application'))
		self.database_ = database.Database(extracted_file_path=str(rospack.get_path('baker_wet_cleaning_application') + "/"))
		#self.database_ = database.Database(extracted_file_path="src/baker/baker_wet_cleaning_application/")
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

		# Document this application run
		# TODO: Allow override from the outside (GUI)
		self.database_.application_data_.last_execution_date_ = datetime.datetime.now()
		self.database_handler_.applyChangesToDatabase()

		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return



		# Find and sort all due rooms
		# ===========================

		# Find due rooms
		self.printMsg("Collecting due rooms...")
		if (self.database_handler_.duePlanningHappenedToday() == True):
			#try:
			self.database_handler_.restoreDueRooms()
			self.database_handler_.getAllDueRooms()
			print len(self.database_.rooms_)
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

		# Sort the due rooms after cleaning method
		self.printMsg("Sorting the found rooms after cleaning method...")
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
		if self.handleInterrupt() == 2:
			return


		
		# Dry cleaning of the due rooms
		# =============================
		self.processDryCleaning(rooms_dry_cleaning, False)
		

		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return



		# Wet cleaning of the due rooms
		# =============================
		self.processWetCleaning(rooms_wet_cleaning, False)
		
		
		# Interruption opportunity
		if self.handleInterrupt() == 2:
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
		if self.handleInterrupt() == 2:
			return
		


		# Wet cleaning of the overdue rooms
		# =================================

		self.processWetCleaning(rooms_wet_cleaning, True)
		
		# Interruption opportunity
		if self.handleInterrupt() == 2:
			return


		
		# COMPLETE APPLICATION
		# ====================

		self.printMsg("Cleaning completed. Overwriting database...")
		#try:
		self.database_handler_.cleanFinished()
		#except:
		#	self.printMsg("Fatal: Database overwriting failed!")
		#	exit(1)
		






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
