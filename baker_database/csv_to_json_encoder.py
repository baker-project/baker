#!/usr/bin/env python

import csv
import datetime
import os

import database
import database_classes


class CSVToJsonEncoder():

	# Required attributes
	csv_file_path_ = ""
	database_file_path = ""
	csv_room_plan_ = None
	csv_territory_plan_ = None 
	database_ = None
	
	
	# Constructor
	def __init__(self, csv_file_path="", database_file_path=""):
		self.csv_file_path_ = csv_file_path
		self.database_ = database.Database(database_file_path)
		#try:
			# There is a database in the specified directory
		self.database_.loadDatabase()
		#except:
			# There is no database in the specified directory
		#	print "There is no database in the specified directory!"
		#	exit(1)
	
	
	# Load a CSV file
	def loadCSVFiles(self):
		# Load the room plan
		file = open(str(self.csv_file_path_) + "ROOMPLAN.csv", "r")
		self.csv_room_plan_ = csv.reader(file, dialect="excel")
		# Load the territory plan
		file = open(str(self.csv_file_path_) + "TERRITORYPLAN.csv", "r")
		self.csv_territory_plan_ = csv.reader(file, dialect="excel")

	# Fill in the database lists
	def feedDatabaseWithCSVData(self):
		# Feed in all the data from the room plan
		i = 0
		existing_rooms = []
		for row in self.csv_room_plan_:
			if (i > 0):
				# Check if the room is already documented and create a new RoomItem otherwise
				pos_id = row[0]
				floor_id = row[1]
				room = self.database_.getRoomByPosFloor(pos_id, floor_id)
				if (room == None):
					room = database_classes.RoomItem()	
					self.database_.rooms_.append(room)
				# Update the data of the concerning RoomItem		
				room.room_position_id_ = pos_id
				room.room_floor_id_ = floor_id
				room.room_building_id_ = row[2]
				room.room_id_ = int(row[3])
				room.room_name_ = row[4]
				room.room_surface_type_ = int(row[6])
				room.room_cleaning_type_ = int(row[7])
				room.room_surface_area_ = float(row[8])
				room.room_trashcan_count_ = int(row[9])
				existing_rooms.append(room.room_id_)
				# Save the database
				self.database_.saveDatabase(temporal=False)	
			i = i + 1
		# Remove all rooms which were not listed in the room plan
		for room in self.database_.rooms_:
			if (not(room.room_id_ in existing_rooms)):
				self.database_.rooms_.remove(room)
			# Save the database
			self.database_.saveDatabase(temporal=False)
		# Feed in all the data from the territory plan
		i = 0
		for row in self.csv_territory_plan_:
			if (i > 1):
				pos_id = row[1]
				floor_id = row[2]
				room = self.database_.getRoomByPosFloor(pos_id, floor_id)
				print room
				room.room_territory_id_ = row[0]
			i = i + 1
			# Save the database
			self.database_.saveDatabase(temporal=False)
		
		# Create assignments from the schedule
		i = 0
		assignment_name_list = ["MO_0", "TU_0", "WE_0", "TH_0", "FR_0", "SA_0", "SU_0", "MO_1", "TU_1", "WE_1", "TH_1", "FR_1", "SA_1", "SU_1"]
		for i in range(len(assignment_name_list)) :
			if (i < 7):
				week_type = 0
				week_day = i
			else:
				week_type = 1
				week_day = i - 7
			current_col = 10 + i
			current_assignment = self.database_.getAssignmentByWeekTypeDay(week_type, week_day)
			if (current_assignment == None):
				current_assignment = database_classes.AssignmentItem()
				self.database_.assignments_.append(current_assignment)
			current_assignment.assignment_name_ = assignment_name_list[i]
			if (i != 0):
				current_assignment.prev_assignment_ = assignment_name_list[i - 1]
			else:
				current_assignment.prev_assignment_ = assignment_name_list[13]
			current_assignment.assignment_week_type_ = week_type
			current_assignment.assignment_week_day_ = week_day
			# Once a file stream went through, it won't work anymore
			# TODO: Find prettier solution
			self.loadCSVFiles()
			j = 0
			for row in self.csv_territory_plan_:
				if (j > 1):
					if (row[current_col] == "X" or row[current_col] == "x"):
						current_assignment.scheduled_rooms_cleaning_.append(self.database_.getRoomByPosFloor(row[1], row[2]).room_id_)
					elif (row[current_col] == "P" or row[current_col] == "p"):
						current_assignment.scheduled_rooms_trashcan_.append(self.database_.getRoomByPosFloor(row[1], row[2]).room_id_)
				j = j + 1
			# Save the database
			self.database_.saveDatabase(temporal=False)
			i = i + 1
	
	
	# Save the database
	def saveDatabaseToFile(self):
		self.database_.saveDatabase(temporal=False)
		
		
	# Public method. This one shall be called. Only this one shall be called.
	def makeDatabase(self):
		self.loadCSVFiles()
		self.feedDatabaseWithCSVData()
		self.saveDatabaseToFile()
		
	

# =========================================================================================
# Test routine
# =========================================================================================

# Initialize and load data from the files
encoder = CSVToJsonEncoder(csv_file_path="csv/", database_file_path="")
encoder.loadCSVFiles()
encoder.feedDatabaseWithCSVData()
