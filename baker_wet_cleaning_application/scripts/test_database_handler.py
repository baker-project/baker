#!/usr/bin/env python

import unittest
import database_utils
import rospkg

from database_handler import DatabaseHandler as DH
from database import Database

PKG = 'baker_wet_cleaning_application'
NAME = 'database_handler_test'
TRASH_TASK = -1
DRY_TASK = 0
WET_TASK = 1


class TestDatabaseHandler(unittest.TestCase):

    def testIsCleaningDay(self):
        self.assertTrue(DH.isCleaningDay('x'))
        self.assertTrue(DH.isCleaningDay('X'))
        self.assertFalse(DH.isCleaningDay('p'))
        self.assertFalse(DH.isCleaningDay(''))

    def testIsTrashDay(self):
        self.assertTrue(DH.isTrashDay('p'))
        self.assertTrue(DH.isTrashDay('P'))
        self.assertFalse(DH.isTrashDay(''))
        self.assertFalse(DH.isTrashDay('x'))

    def testIsDryDay(self):
        self.assertTrue(DH.isDryDay(2))
        self.assertTrue(DH.isDryDay(0))
        self.assertFalse(DH.isDryDay(1))

    def testIsWetDay(self):
        self.assertTrue(DH.isWetDay(2))
        self.assertTrue(DH.isWetDay(1))
        self.assertFalse(DH.isWetDay(0))

    def testIsTrashDay(self):
        self.assertTrue(DH.isTrashDay(0))
        self.assertTrue(DH.isTrashDay(1))
        self.assertTrue(DH.isTrashDay(2))

    def testRestoreDueRooms(self):

        scenarios = [{},
                     {'open_cleaning_tasks': {}},
                     {'open_cleaning_tasks': {
                         '1': [TRASH_TASK, DRY_TASK],
                         '8': [WET_TASK, TRASH_TASK],
                         '4': [TRASH_TASK]
                     }}]

        outputs = [set([]), set([]), set([4, 1, 8])]

        rospack = rospkg.RosPack()
        for k in range(len(scenarios)):
            database_location = str(rospack.get_path('baker_wet_cleaning_application') + '/resources')
            database_utils.updateDatabaseToScenario(scenarios[k], database_location=database_location + '/json/')
            database = Database(extracted_file_path=database_location)
            database_handler = DH(database)
            database_handler.restoreDueRooms()
            database_handler.computeAllDueRooms()
            due_rooms = database_handler.due_rooms_
            due_rooms_ids = [room.room_id_ for room in due_rooms]
            print(set(due_rooms_ids), outputs[k])
            self.assertEqual(set(due_rooms_ids), outputs[k], msg="Test index {}".format(k))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestDatabaseHandler)
