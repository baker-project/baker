#!/usr/bin/env python

import database

class JSONToCSVEncoder():

    # Required attributes
    csv_file_path_ = ""
    database_file_path_ = "" # This must be the path of the folder where the "resources" folder is contained.
    database_ = None

    
    # Constructor. Requires path of the database files and the wanted path of the CSV files.
    def __init__(csv_file_path="", database_file_path=""):
        self.database_ = database.Database(extracted_file_path=database_file_path)
        self.database_.loadDatabase()
        self.csv_file_path_ = csv_file_path
    

    # Method to create a room book out of the database files.
    def createRoomBook(self):
        file = open(str(self.csv_file_path_) + "/csv/ROOMBOOK.csv", "w")
        for room in self.database_.rooms_:
            file.write(str(room.room_position_id_) + ";")
            file.write(str(room.room_floor_id_) + ";")
            file.write(str(room.room_building_id_) + ";")
            file.write(str(room.room_id_) + ";")
            file.write(str(room.room_name_) + ";")
            file.write("Reinigungsbereich;")
            file.write(str(room.room_surface_type_) + ";")
            file.write(str(room.room_cleaning_method_) + ";")
            file.write(str(room.room_surface_area_) + ";")
            file.write(str(room.room_trashcan_count_) + ";")
            file.write(";")
            file.write("?;")
            file.write("?;")
            file.write("?;")
            file.write("?;")
            file.write("Woche;")
            file.write("\n")
        file.close()


    # Method to create a terrytory plan out of the database files.
    def createTerritoryPlan(self):
        file = open(str(self.csv_file_path_) + "/csv/TERRITORYPLAN.csv", "w")
        for room in self.database_.rooms_:
			file.write(str(room.room_territory_id_) + ";")
			file.write(str(room.room_position_id_) + ";")
			file.write(str(room.room_floor_id_) + ";")
			file.write(str(room.room_id_) + ";")
			file.write("Bezeichnung 1;")
			file.write("Bezeichnung 2;")
			file.write("Raumgruppe;")
			file.write("Bezeichnung 3;")
			file.write(str(room.room_surface_type_) + ";")
			file.write(str(room.room_surface_area_) + ";")
			file.write("INTERVAL_STRING;")
			file.write(str(";".join(room.room_scheduled_days_)) + ";")
			file.write("\n")
        file.close()


    # Public method which creates a roombook and a territory plan from the stated database files. 
    # Call this method from the outside.
    def createCSVFiles(self):
        self.createRoomBook()
        self.createTerritoryPlan()
