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
            pass


    # Method to create a terrytory plan out of the database files.
    def createTerritoryPlan(self):
        file = open(str(self.csv_file_path_) + "/csv/TERRITORYPLAN.csv", "w")
        for room in self.database_.rooms_:
            pass


    # Public method which creates a roombook and a territory plan from the stated database files. 
    # Call this method from the outside.
    def createCSVFiles(self):
        self.createRoomBook()
        self.createTerritoryPlan()
