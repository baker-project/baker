============================
BakeR database documentation
Last updated: 21st July 2018
Author: rmb-mf
============================

Contents of this folder:
========================

	database.py				Contains the database class. Update this file if necessary. Current version is inside "/baker_wet_cleaning_application/scripts".
	database_classes.py			Contains all structures used in database.py. Update this file if necessary. Current version is inside "/baker_wet_cleaning_application/scripts".
	database_creation_tool.py		Creates a database from the data available in the ROS action servers. Requires map receiving and segmentation ROS servers running.
	json_log_to_csv_log_converter.py	Reads a specified log json file and creates a CSV file containing the data in a preferred manner.
	plan_to_json_converter.py		Reads the room and territory plan CSV files and fills a previously created database set with the contained data.
	json_to_plan_converter.py		Reads the database set files and restores a roombook and territory plan from the provided data.

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
			- def creatrTerritoryPlan(): Creates a territory plan from the result of def runDatabaseCreation()
			