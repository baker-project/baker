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
		## Load the territory plan
		#file = open(str(self.csv_file_path_) + "TERRITORYPLAN.csv", "r")
		#self.csv_territory_plan_ = csv.reader(file, dialect="excel")

	# Fill in the database lists
	def feedDatabaseWithCSVData(self):
		# Feed in all the data from the room plan
		# Index i ensures that the title row will be ignored
		i = 0
		existing_rooms = []
		for row in self.csv_room_plan_:
			if (i > 0):
				# Check if the room is already documented and create a new RoomItem otherwise
				room_id = int(row[3])
				room = self.database_.getRoom(room_id)
				if (room == None):
					# Actually, an error should be risen, but for now...
					print "ROOM WITH ID " + str(room_id) + " DOES NOT EXIST!"
					room = database_classes.RoomItem()
					room.room_cleaning_datestamps_ = [None, None, None]	
					self.database_.rooms_.append(room)
				# Update the data of the concerning RoomItem		
				room.room_position_id_ = row[0]
				room.room_floor_id_ = row[1]
				room.room_building_id_ = row[2]
				room.room_id_ = room_id
				room.room_name_ = row[4]
				room.room_surface_type_ = int(row[6])
				room.room_cleaning_type_ = int(row[7])
				room.room_surface_area_ = float(row[8])
				room.room_trashcan_count_ = int(row[9])
				scheduled_days = []
				for day in range(14):
					scheduled_days.append(row[16 + day])
				room.room_scheduled_days_ = scheduled_days
				existing_rooms.append(room.room_id_)
				# Save the database
				self.database_.saveRoomDatabase(temporal=False)	
			i = i + 1
		# Remove all rooms which were not listed in the room plan
		for room in self.database_.rooms_:
			if (not(room.room_id_ in existing_rooms)):
				self.database_.rooms_.remove(room)
			# Save the database
			self.database_.saveRoomDatabase(temporal=False)
		
	
	
	# Save the database
	def saveDatabaseToFile(self):
		self.database_.saveRoomDatabase(temporal=False)
		
		
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
encoder.makeDatabase()
