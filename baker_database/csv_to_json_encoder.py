#!/usr/bin/env python

import csv
import datetime

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
	def __init__(self, csv_file_path="", database_file_path="", old_database=None):
		self.csv_file_path_ = csv_file_path
		# Do different things depending on if this is an update or a creation
		if (old_database == None):
			self.database_file_path_ = database_file_path
			self.database_ = database.database(self.database_file_path_)
		else:
			self.database_ = old_database
			self.database_file_path_ = self.database_.extracted_file_path
	
	
	# Load a CSV file
	def loadCSVFiles(self):
		# Load the room plan
		file = open(str(self.csv_file_path) + "ROOMPLAN.csv", "r")
		self.csv_room_plan_ = csv.reader(file, dialect="excel")
		# Load the territory plan
		file = open(str(self.csv_file_path) + "TERRITORYPLAN.csv", "r")
		self.csv_territory_plan_ = csv.reader(file, dialect="excel")
	
	
	# Fill in the database lists
	def feedDatabaseWithCSVData(self):
		# Feed in all the data which is assigned to the rooms
		for row in self.csv_room_plan_:
			pass
		# Feed in all the data which is assigned to the assignments
		for row in self.csv_territory_plan_:
			pass
	
	
	# Save the database
	def saveDatabaseToFile(self):
		self.database_.saveDatabase()
		
		
	# Public method. This one shall be called. Only this one shall be called.
	def makeDatabase(self):
		self.loadCSVFiles()
		self.feedDatabaseWithCSVData()
		self.saveDatabaseToFile()
		
	