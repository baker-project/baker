============================
BakeR database documentation
Last updated: 6th July 2018
Author: rmb-mf
============================

Contents of this folder:
	database.py				Contains the database class. Update this file if necessary. Current version is inside "/baker_wet_cleaning_application/scripts".
	database_classes.py			Contains all structures used in database.py. Update this file if necessary. Current version is inside "/baker_wet_cleaning_application/scripts".
	database_creation_tool.py		Creates a database from the data available in the ROS action servers. Requires map receiving and segmentation ROS servers running.
	json_log_to_csv_log_converter.py	Reads a specified log json file and creates a CSV file containing the data in a preferred manner.
	plan_to_json_converter.py		Reads the room and territory plan CSV files and fills a previously created database set with the contained data.

Subfolders of this folder:
	/csv					Contains the room plan and territory plan CSV files.
	/resources				This is the database set which was created using database_creation_tool.py

