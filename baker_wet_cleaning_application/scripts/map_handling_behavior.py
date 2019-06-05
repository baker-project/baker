#!/usr/bin/env python

from room_sequencing_behavior import RoomSequencingBehavior
import behavior_container


class MapHandlingBehavior(behavior_container.BehaviorContainer):

	# ========================================================================
	# Description:
	# Returns a sorted list of RoomInformation from a given 
	# database_classes.RoomItem list
	# ========================================================================
	
	# Method for setting parameters for the behavior
	def setParameters(self, database_handler, rooms_list):
		# Parameters set autonomously
		self.room_sequencing_service_str_ = '/room_sequence_planning/room_sequence_planning_server'
		# Parameters set from the outside
		self.database_handler_ = database_handler
		self.rooms_list_ = rooms_list
		self.is_finished = False


	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no data to be saved
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		
		# Get the parameters for room sequencing
		(self.room_information_in_pixel_, self.segmented_map_) = self.database_handler_.getMapAndRoomInformationInPixel(self.rooms_list_)
	
		# Interruption opportunity
		if self.handleInterrupt() >= 1:
			return

		# Room sequencing
		self.room_sequencer_ = RoomSequencingBehavior("Room sequencing", self.interrupt_var_, self.room_sequencing_service_str_)
		self.room_sequencer_.setParameters(
			database=self.database_handler_.database_,
			room_information_in_pixel=self.room_information_in_pixel_,
			robot_radius=self.database_handler_.database_.robot_properties_.exploration_robot_radius_
			)
		self.room_sequencer_.executeCustomBehavior()

		self.room_sequencing_data_ = self.room_sequencer_.room_sequence_result_
		self.mapping_ = self.database_handler_.getRoomMapping(self.rooms_list_, self.room_sequencing_data_)
		#self.printMsg("self.room_sequencing_data_.checkpoints=" + str(self.room_sequencing_data_.checkpoints))
		#self.printMsg("self.segmentation_data_.room_information_in_pixel=" + str(self.segmentation_data_.room_information_in_pixel))
		#self.printMsg("self.segmentation_data_.room_information_in_meter=" + str(self.segmentation_data_.room_information_in_meter))