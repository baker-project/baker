================================================================================================
BakeR database utilities documentation
Last updated: 24st July 2018
Author: rmb-mf
================================================================================================

Contents of this folder:
========================

	database.py							Contains the database class. Update this file if changed somewhere. Current version is inside "/baker_wet_cleaning_application/scripts".
	database_classes.py					Contains all structures used in database.py. Update this file if changed somewhere. Current version is inside "/baker_wet_cleaning_application/scripts".
	database_creation_tool.py			Creates a database from the data available in the ROS action servers. Requires map receiving and segmentation ROS servers running.
	json_log_to_csv_log_converter.py	Reads a specified log json file and creates a CSV file containing the data in a preferred manner.
	plan_to_json_converter.py			Reads the room and territory plan CSV files and fills a previously created database set with the contained data.
	json_to_plan_converter.py			Reads the database set files and restores a roombook and territory plan from the provided data.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
THE DOCUMENTATION FOR database.py, database_handler.py AND database_classes.py IS LOCATED AT 
"[...]/src/baker_baker_wet_cleaning_application/scripts/readme.txt"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Subfolders of this folder:
==========================

	/csv					Contains the room plan and territory plan CSV files.
	/resources				This is the database set which was created using database_creation_tool.py


Use cases:
==========

	1.	Creating a whole new database set:
		- run database_creation_tool.py
		  --> construction of database file systems and storage of all data which could be retreived by the ROS action servers
		- run plan_to_json_converter.py
		  --> the previously created database is now filled also with the data provided by the CSV files

	2.	Updating a database set with new CSV data:
		- run plan_to_json_converter.py
		  --> all information from the CSV files will be overwritten by the new data

	3.	Updating anything except of CSV data:
		(not possible, apply use case 1)

	4. Creating a room book and a territory plan CSV file from a database set
		- run json_to_plan_converter.py
		--> the CSV files will be generated

File information:
=================

	database_creation_tool.py
		WHAT IT DOES:
			- Creates a new database set
			- Can create an empty (i.e. unfilled) roombook and territory plan if wanted.
		REQUIREMENTS FOR USAGE:
			- Map receiving action server
			- Map segmentation action server
		USAGE:
			- Create new DatabaseCreator instance
			- Run def runDatabaseCreation()
			- If you want an empty roombook, run def createRoomBook()
			- If you want an empty territory plan, run def creatrTerritoryPlan()
		METHOD INFORMATION:
			- def runDatabaseCreation(): Creates JSON files, file structure and map images
			- def createRoomBook(): Creates a roombook from the result of def runDatabaseCreation()
			- def createTerritoryPlan(): Creates a territory plan from the result of def runDatabaseCreation()



	plan_to_json_converter.py
		WHAT IT DOES:
			- Reads a room book CSV file and a territory plan CSV file
			- Inserts the relevant data from those files into a previously created database set
		REQUIREMENTS FOR USAGE:
			- Existing database set
			- Complete CSV Room plan with
			- Complete CSV Territory plan
			- File paths for database "resource" folder
			- File paths for folder containing CSV files
		USAGE:
			- Create new CSVToJSONEncoder instance. Parameters are the file paths stated above.
			- Run def makeDatabase()
		METHOD INFORMATION:
			- def __init__(self, csv_file_path="", database_file_path=""): Creates a new CSVToJSONEncoder instance. The paths of the database "recources" folder and the CSV files can be specified.
			- def loadCSVFiles(self, room_plan_name = "ROOMPLAN.csv", territory_plan_name = "TERRITORYPLAN.csv"): Loads the CSV files from the directory stated at the construction of the object. The filenames of the CSV files can be specified.
			- def feedDatabaseWithCSVData(): Inserts the contents of the loaded CSV files into the database set which is located at the path stated before.
			- def saveDatabaseToFile(): Applies the changes to the database set and saves the altered files on disk.
			- def makeDatabase(self, room_plan_name = "ROOMPLAN.csv", territory_plan_name = "TERRITORYPLAN.csv"): Runs the above three methods in the correct order. The filenames of the CSV files can be specified.



	json_to_plan_converter.py
		WHAT IT DOES:
			- Reads a database set
			- Creates a room book and a territory plan CSV file out from the data
		REQUIREMENTS FOR USGE:
			- Existing database set
			- File paths for database "resource" folder
			- File paths for folder where the CSV files should be created in.
		USAGE:
			- Create JSONToCSVEncoder instance. Parameters of the file paths stated above.
			- Run def createCSVFiles()
		METHOD INFORMATION:
			- def __init__(self, csv_file_path="", database_file_path=""): Creates a new JSONToCSVEncoder instance. The paths of the database "recources" folder and the CSV files can be specified.
			- def createRoomBook(): Creates a new room book CSV file from the database and saves it at path which was stated before. File name can be specified.
			- def createTerritoryPlan(): Creates a new territory plan CSV file from the database and saves it at path which was stated before. File name can be specified.
			- def createCSVFiles(): Runs the upper two methods. File names of the resulting CSV files can be specified.