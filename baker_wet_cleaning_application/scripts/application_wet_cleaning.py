#!/usr/bin/env python

import rospy
import rospkg
import actionlib
import application_container
from map_handling_behavior import MapHandlingBehavior
from dry_cleaning_behavior import DryCleaningBehavior
from wet_cleaning_behavior import WetCleaningBehavior
import database
import database_handler

from geometry_msgs.msg import Point32
import datetime

class WetCleaningApplication(application_container.ApplicationContainer):
	# ========================================================================
	# Description:
	# Highest element in the hierarchy of the cleaning application
	# ========================================================================

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
			room_information_in_meter=self.database_handler_.getRoomInformationInMeter(rooms_dry_cleaning),
			robot_frame_id=self.robot_frame_id_
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
			sequencing_result=self.map_handler_.room_sequencing_data_,
			mapping=self.map_handler_.mapping_,
			robot_frame_id=self.robot_frame_id_,
			robot_radius=self.robot_radius_,
			coverage_radius=self.coverage_radius_,
			field_of_view=self.field_of_view_,
			field_of_view_origin=self.field_of_view_origin_,
			use_cleaning_device=self.use_cleaning_device_  # hack: cleaning device can be turned off for trade fair show
		)
		self.wet_cleaner_.executeBehavior()

	def computeAndSortDueRooms(self, shall_continue_old_cleaning):
		self.printMsg("Collecting due rooms...")
		self.database_handler_.due_rooms_ = []
		if self.database_handler_.noPlanningHappenedToday() and not shall_continue_old_cleaning:
			self.database_handler_.computeAllDueRooms()
			print "len(self.database_.rooms_):", len(self.database_.rooms_), "\ndue rooms:"
			for room in self.database_handler_.due_rooms_:
				print(room.room_name_)
		else:
			self.database_handler_.restoreDueRooms()

		# Sort the due rooms with respect to cleaning method
		self.printMsg("Sorting the found rooms with respect to cleaning method...")
		(rooms_dry_cleaning, rooms_wet_cleaning) = self.database_handler_.sortRoomsList(self.database_handler_.due_rooms_)
		for room in rooms_dry_cleaning:
			self.printMsg(str(room.room_name_) + " ---> DRY")
		for room in rooms_wet_cleaning:
			self.printMsg(str(room.room_name_) + " ---> WET")

		# Document completed due rooms planning in the database
		self.database_.application_data_.last_planning_date_[0] = datetime.datetime.now()
		self.database_handler_.applyChangesToDatabase()

		return rooms_dry_cleaning, rooms_wet_cleaning

	# checks if all these rooms are due rooms are cleaned in the database
	# throws exception if not
	def checkAbstractProcessCleaning(self, rooms_cleaning, cleaning_method):
		print("checking for cleaning_method{}".format(cleaning_method))
		cleaning_duration_tolerance = 24*60  # in minutes
		now = datetime.datetime.now()

		for room in rooms_cleaning:
			last_cleaning_date = room.room_cleaning_datestamps_[cleaning_method]
			if now - datetime.timedelta(minutes=cleaning_duration_tolerance) > last_cleaning_date:
				# todo (rmb-ma) find the best way to handle exceptions for the application
				raise RuntimeError("Room {} supposed to be cleaned (cleaning_method {}) but it's not (last_cleaning_date={})"
								   .format(room.room_id_, cleaning_method, last_cleaning_date))

	def checkProcessWetCleaning(self, rooms_cleaning):
		self.checkAbstractProcessCleaning(rooms_cleaning, cleaning_method=2)

	def checkProcessDryCleaning(self, rooms_cleaning):
		# warning. only trash ? Trash + dirt ?
		self.checkAbstractProcessCleaning(rooms_cleaning, cleaning_method=1)

	def checkNoUndoneCleaningTasks(self):
		# todo (rmb-ma)
		pass

	# Implement application procedures of inherited classes here.
	def executeCustomBehavior(self, last_execution_date_override=None):

		# Initialize behaviors
		# ====================
		self.map_handler_ = MapHandlingBehavior("MapHandlingBehavior", self.application_status_)
		self.dry_cleaner_ = DryCleaningBehavior("DryCleaningBehavior", self.application_status_)
		self.wet_cleaner_ = WetCleaningBehavior("WetCleaningBehavior", self.application_status_)

		# Load data from the robot device
		# ===============================

		# todo: read out these parameters
		if rospy.has_param('robot_frame'):
			self.robot_frame_id_ = rospy.get_param("robot_frame")
			self.printMsg("Imported parameter robot_frame = " + str(self.robot_frame_id_))
			# todo: write into database ??? (rmb-ma)
		else:
			self.robot_frame_id_ = 'base_link'
			self.printMsg("Parameter robot_frame_id assigned to default value '{}'".format(self.robot_frame_id_))

		if rospy.has_param('robot_radius'):
			self.robot_radius_ = rospy.get_param("robot_radius")
			self.printMsg("Imported parameter robot_radius = " + str(self.robot_radius_))
		else:
			self.robot_radius_ = 0.325
			self.printMsg("Parameter robot_radius assigned to default value '{}'".format(self.robot_radius_))

		if rospy.has_param('coverage_radius'):
			self.coverage_radius_ = rospy.get_param("coverage_radius")
			self.printMsg("Imported parameter robot_radius = " + str(self.coverage_radius_))
		else:
			self.printMsg("Parameter coverage radius doesn't exist")
			return

		# todo: get field_of_view

		#self.robot_frame_id_ = 'base_link'
		#self.robot_radius_ = 0.2875  #0.325	# todo: read from MIRA
		#self.coverage_radius_ = 0.233655  #0.25	# todo: read from MIRA
#		self.field_of_view_ = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364),
#							   Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)]	# todo: read from MIRA
		self.field_of_view_ = [Point32(x=0.080, y=0.7), Point32(x=0.080, y=-0.7),
							   Point32(x=2.30, y=-0.7), Point32(x=2.30, y=0.7)]	# todo: read from MIRA
		self.field_of_view_origin_ = Point32(x=0.0, y=0.0) # todo: read from MIRA

		# todo: hack: cleaning device can be turned off for trade fair show
		self.use_cleaning_device_ = False
		if rospy.has_param('use_cleaning_device'):
			self.use_cleaning_device_ = rospy.get_param("use_cleaning_device")
			self.printMsg("Imported parameter use_cleaning_device = " + str(self.use_cleaning_device_))

		# Load database, initialize database handler
		# ==========================================

		# Initialize and load database
		self.printMsg("Loading database from files...")
		rospack = rospkg.RosPack()
		print str(rospack.get_path('baker_wet_cleaning_application'))
		self.database_ = database.Database(extracted_file_path=str(rospack.get_path('baker_wet_cleaning_application') + "/resources"))
		self.database_handler_ = database_handler.DatabaseHandler(self.database_)

		if self.database_.application_data_.last_planning_date_ is None:
			self.database_.application_data_.last_planning_date_ = datetime.datetime(datetime.MINYEAR, 1, 1)
		if self.database_.application_data_.last_execution_date_ is None:
			self.database_.application_data_.last_execution_date_ = datetime.datetime(datetime.MINYEAR, 1, 1)
		days_delta = datetime.datetime.now() - self.database_.application_data_.last_execution_date_
		print("-- CURRENT_DATE: " + str(datetime.datetime.now()))
		print("-- LAST_EXECUTION_DATE: " + str(self.database_.application_data_.last_execution_date_))
		print("-- LAST_PLANNING_DATE_0: " + str(self.database_.application_data_.last_planning_date_[0]))
		print("-- DAYS_DELTA: " + str(days_delta) + " " + str(days_delta.days))
		print("-- PLANNING_OFFSET: " + str(self.database_.application_data_.planning_offset_))

		shall_continue_old_cleaning = self.database_.application_data_.progress_[0] == 1 and days_delta.days == 0
		if self.database_.application_data_.progress_[0] == 1 and days_delta.days != 0:
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

		if self.handleInterrupt() >= 1:
			return

		# Find and sort all due rooms
		# ===========================
		(rooms_dry_cleaning, rooms_wet_cleaning) = self.computeAndSortDueRooms(shall_continue_old_cleaning)

		if self.handleInterrupt() >= 1:
			return

		# Dry cleaning of the due rooms
		# =============================
		self.processDryCleaning(rooms_dry_cleaning, is_overdue=False)
		if self.handleInterrupt() >= 1:
			return

		self.checkProcessDryCleaning(rooms_dry_cleaning)
		if self.handleInterrupt() >= 1:
			return

		# Wet cleaning of the due rooms
		# =============================
		self.processWetCleaning(rooms_wet_cleaning, is_overdue=False)
		if self.handleInterrupt() >= 1:
			return

		self.checkProcessWetCleaning(rooms_wet_cleaning)
		if self.handleInterrupt() >= 1:
			return

		# Find and sort all overdue rooms
		# ===============================
		self.database_handler_.computeAllOverdueRooms()
		(rooms_dry_cleaning, rooms_wet_cleaning) = self.database_handler_.sortRoomsList(self.database_handler_.overdue_rooms_)

		# Document completed due rooms planning in the database
		self.database_.application_data_.last_planning_date_[1] = datetime.datetime.now()
		self.database_handler_.applyChangesToDatabase()

		# Dry cleaning of the overdue rooms
		# =================================
		#self.processDryCleaning(rooms_dry_cleaning, is_overdue=True)
		self.checkProcessDryCleaning(rooms_dry_cleaning)

		if self.handleInterrupt() >= 1:
			return

		# Wet cleaning of the overdue rooms
		# =================================
		#self.processWetCleaning(rooms_wet_cleaning, is_overdue=True)
		self.checkProcessWetCleaning(rooms_wet_cleaning)

		if self.handleInterrupt() >= 1:
			return

		# Complete application
		# ====================
		self.printMsg("Cleaning completed. Overwriting database...")
		self.database_.application_data_.progress_ = [0, datetime.datetime.now()]
		self.database_handler_.cleaningFinished()

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
