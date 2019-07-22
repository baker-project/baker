
#BAKER WET CLEANING APPLICATION DOCUMENTATION

Last updated: _22th July 2019_
Authors: _rmb-mf_, _rmb-ma_

**Contents:**
	- BakeR database documentation
	- BakeR wet cleaning application: Getting started

----

## BakeR database documentation

###Contents which will be discussed in this section:

* `database.py`: Clusters all contents of the database in a Database object, contains methods to load and save them safely and contains methods for logging the progress of the robot.
* `database_classes.py`:	 Contains definitions of the objects which are stored in the database
* `database_handler.py`:	 Contains all methods for editing the database, in particular also for calculating things from the data the database provides


### File information:

#### `database.py`:
##### **What it does**:
* Contains all data of a database set on the disk such that the data can be accessed by Python scripts
* Synchronizes the contents of a database instance with the files saved on a disk in a safe way, in order to avoid data loss
* Creates and updates log files which document the progress of cleaning
##### **Requirements for usage:**
* database file set on disk (into `/resources` folder)
##### **Basic function:**
* **loading**:
When loading the contents from a database set on the disk, database tries to find an intact file set. This will be a temporal file set if there is a complete one.
Having loaded the JSON files, dictionaries will be created based on the file stream.
The contents of the dictionaries are then first converted into a suitable format (e.g. `date string --> datetime.Datetime object`) and then fed into instances of the classes of `database_classes.py`.
* **saving**:
Depending on whether it should be saved as temporal version or not, temporal files will be created or overwritten.
From the attributes contained in database, dictionaries are created. Therefore, data types unsuitable for JSON are converted (e.g. `datetime.Datetime object --> string`).
A string which is of JSON syntax will be created based on the dictionaries and saved into the designated files.
* **Adding a log entry**:
database loads the correct log JSON file in a similar manner to the loading routine described above. If there is no such file, a new one will be created.
	* database _creates_ a backup file, which contains the same data as the loaded file.
	* database _adds_ the wanted log entry to the loaded log.
	* database _saves_ the updated log into the original file.
	* database _removes_ the backup file.
##### **Usage:**
* If you want to **have a loaded Database** object
	1. Initialize database instance. State a file path, if it is not in the same directory as the location of `database.py`.
	2. Run `loadDatabase()`
* If you want to **save a loaded Database** object
	  Run `saveCompleteDatabase()`. Depending on whether you want so save it temporarily or finally, state "`temporal_file=<True/False>`".
* If you want to **add a new log entry**
	  Run `addLogEntry(log_element)`. log_element must be the logItem object you want to add. (_Better use_ `database_handler.DatabaseHandler.addLogEntry()`)
			- If you want to **delete all temporal files** in order to let the robot forget everything that just happened
			  `discardTemporalDatabase()`
##### Method information:
* `@staticmethod <XXX>`: Converts between datatypes, such as string, datetime, point32, array.
* `def update<XXX>(self, dict)`: Reads the provided dict and adds the [converted] elements into the database_classes attributes of the database.
* `def get<XXX>dictFrom<XXX>(self)`: Returns a dict containing the [converted] attributes of the database_classes attributes of the database.
* `def readFiles(self, temporal)`: Internal method for loading the JSON files. Public shall call def `loadDatabase()`.
* `def checkIntegrity(self, temporal)`: Internal method for checking whether a database set is damaged or not by trying to check certain database entries.
* `def save<RoomDatabase/GlobalApplicationData>(self, temporal=True):` Method which saves the rooms list of the database / the global application data.
* `def updateRunCount(self, date):` Updates the amount of application executions per day. Parameter date must be the wanted date as datetime.Datetime.
* `def getCurrentLogfileName(self):` Returns the file name of the current log file.
* `def addLogEntry(self, log):` Adds a log entry in the current log file. Parameter log must be the LogItem instance to be added.
* `def discardTemporalDatabase(self):` Deletes all temporal files without saving their content. Also sets the application prograss variable to 4 (i.e. DISCARDED).
* `def loadDatabase(self):` Method to real all database related files on the disk.
* `def saveCompleteDatabase(self, temporal_file=True):` Method to save all entries of database in files on the disk. Parameter temporal_file indicates whether the original or temporal files are overwritten.
* `def getRoom(self, room_id):` Method which returns a pointer to the RoomItem with room ID room_id.


#### `database_classes.py`
##### What it does:
* Contains definitions for classes, which resemble the attributes of Database
				- RobotProperties
				- GlobalSettings
				- GlobalMapData
				- GlobalApplicationData
				- LogItem
				- RoomIssue
				- RoomItem
##### Requirements for usage:
(None)
##### Usage:
(Internal only, none relevant for public)
##### Method information:
(Internal only, none relevant for public)

#### `database_handler.py`
##### What it does:
* Reads a Database object and creates a list of due and overdue rooms
* Writes timestamps in the database
* Restores a cleaning progress after sudden stop of program
* Converts contents of the database into a format suitable for the existing action servers
##### Requirements for usage:
* Database instance
#### Usage:
* If you want a list of all due rooms
  Run `def getAllDueRooms()`
* If you want a list of all overdue rooms
  Run` def getAllOverdueRooms()`
* If you want to declare a cleaning subtask to be finished
  Run `def checkoutCompletedRoom(room, assignment_type)`. See `database_classes.py` for definition of assignment_type.
##### Method information:
* `@staticmethod <XXX>`: Returns information on the current day such as week type, week day, schedule index.
* `def getMapAndRoomInformationInPixel(self, rooms_array)`: Method which returns a map and `RoomInformation` instances (in Pixel) out of an array of `RoomItem` instances.
* `def getRoomInformationInMeter(self):` Method which returns `RoomInformation` instances (in Meter) out of an array of RoomItem instances.
* `def getRoomMapping(self, rooms_list, room_sequence_result):` Method which creates a mapping that maps the room sequence order to the` RoomItem.room_id_.`
* `def getAllDueRooms(self):` Method which adds the due room cleaning tasks in the corresponding `RoomItem` instances of database. Affected `RoomItem` instances will be added in the due_rooms_ array of database_handler.
* `def restoreDueRooms(self)`: Method which looks for `RoomItem` instances of database which do have any open cleaning task and collects them in a list.
* `def getAllOverdueRooms(self):` Method which adds the overdue room cleaning tasks in the corresponding `RoomItem` instances of database. Affected `RoomItem` instances will be added in the overdue_rooms_ array of database_handler.
* `def noPlanningHappenedToday(self)`: Method that returns a boolean value whether a planning process was performed today already.
* `def sortRoomsList(self, rooms_list)`: Method that creates two arrays out of rooms_list. The first array contains all the rooms which must be cleaned dry and the ons which only need empty trashcans. The second array contains all the rooms which need to be cleaned wet. In general, the two arrays are not disjunct.
* `def checkoutCompletedRoom(self, room, assignment_type)`: Method that updates the corresponding time stamp of a specified room and removes the specified assignment from its open cleaning tasks.
* `def addLogEntry(self, ...)`: Method that creates a new `LogItem` instance out of the provided parameters and saves it in the current log file.

-----
## BakeR wet cleaning application: Getting started

The uppermost script, concerning the wet cleaning application, is application_wet_cleaning.py.
Any other script, concerning the wet cleaning application, will be called from there.


### Basic function of `application_wet_cleaning.py` (ordered)

1. Loads database
2. Retrieves all due rooms and sorts them after cleaning method
3. Performs dry cleaning process of due rooms
4. Check for all due rooms if they were cleaned (look at the database entries)
5. Performs wet cleaning process of due rooms
6. Check for all due rooms if they were cleaned (look at the database entries)
7. Retrieves all overdue rooms and sorts them after cleaning method
8. Performs dry cleaning process of overdue rooms
9. Check for all overdue rooms if they were cleaned (look at the database entries)
10. Performs wet cleaning process of overdue rooms
11. Check for all overdue rooms if they were cleaned (look at the database entries)

### Structure of the application

```
application_wet_cleaning.py
├── database.py
│   └── database_classes.py
├── database_handler.py
├── map_handling_behavior.py
│   └── room_sequencing_behavior.py
└── abstract_cleaning_behavior.py
    ├── move_base_behavior.py: to move into the correct room
    ├── room_exploration_behavior.py: to compute the coverage path
    ├── tool_changing_behavior.py (unused)
    ├── trolley_moving_behavior.py (unused)
    ├── dry_cleaning_behavior.py
    |   ├── move_base_path_behavior.py: to follow the coverage path
    │   ├──  dirt_removing_behavior.py
    |   |    ├── move_base_path_behavior.py: to move to the dirt position
    |   |    ├── uses VacuumCleaner Interface
    |   |    └── uses CheckPerimeterAccessibility
    |   └── trashcan_emptying_behavior.py
    |        ├── move_base_path_behavior.py
    |        └── uses Arm Module Interface
    ├── wet_cleaning_behavior.py
    |   ├── move_base_path_behavior.py: to follow the coverage path
    |   └── move_base_wall_follow_behavior.py: to follow the wall at the end of the cleaning
    └── uses CheckCoverageMonitor (to compute the cleaned area)
```
