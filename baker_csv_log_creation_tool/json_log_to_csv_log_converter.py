#!/usr/bin/env python

import database
import json


class JSONLogToCSVLogConverter():

	# Initialize converter; define paths of source and destination file
	def __init__(self, json_file_path="", csv_file_path=""):
		self.json_file_path_ = json_file_path
		self.csv_file_path_ = csv_file_path
		self.database_ = database.Database()

	# Load JSON log file
	def loadJSONLog(self, filename):

		# Load database
		if (os.path.isfile(str(self.json_file_path_) + str(filename)) == True):
			file = open(str(self.json_file_path_) + str(filename), "r").read()
			# Translate text to dict and dict to list of LogItem
			log_item_dict = json.loads(file)
			self.log_item_list_ = self.database_.getLogListFromLogDict(log_item_dict)
			self.filename_ = filename
		else:
			print "ERROR: FILE DOES NOT EXIST"
			exit(1)

	# Create string which can be recognizes as a time
	def createTimeString(self, date_and_time):
		return(str(date_and_time.hour) + ":" + str(date_and_time.minute))

	# Create a readable clenaing task expression [GERMAN]
	def createCleanintTaskString(self, task):
		if (task == -1):
			return "Papierkorb"
		elif (task == 0):
			return "Trockenreinigung"
		elif (task == 1):
			return "Nassreinigung"
		else:
			return ""

	# Create a readable status expression [GERMAN]
	def createStatusString(self, status):
		if (status == 0):
			return "Erfolg"
		elif (status == 1):
			return "Fehlerhaft"
		else:
			return ""

	# Create a comma text of all problem IDs
	def createProblemIDString(self, ids):
		result = ""
		for i in len(ids) - 2:
			result = result + str(ids(i)) + ", "
		result = result + ids[len(ids) - 1]
		return result

	
	# Create CSV file
	def saveCSVLog(self):
		file = open(str(self.json_file_path_) + str(self.filename_), "w")
		file.write("")
		file = open(str(self.json_file_path_) + str(self.filename_), "a")
		for log_item in self.log_item_list_:
			file.write(str(log_item.room_id_) + ";")
			file.write(str(self.createTimeString(log_item.date_and_time_)) + ";")
			file.write(str(self.createCleanintTaskString(log_item.cleaning_task_)) + ";")
			file.write(str(log_item.found_trashcans_) + ";")
			file.write(str(log_item.cleaned_surface_area_) + ";")
			file.write(str(log_item.found_dirtspots_) + ";")
			file.write(str(log_item.battery_usage_) + ";")
			file.write(str(log_item.used_water_amount_) + ";")
			file.write(str(log_item.trolley_capacity_) + ";")
			file.write(str(self.createStatusString(log_item.status_)) + ";")
			file.write(str(self.createProblemIDString(log_item.room_issues_)) + ";")
			file.write("\n")


	# Method to call from the outside
	def runConverter(self, filename):
		self.loadJSONLog(filename)
		self.saveCSVLog()


		