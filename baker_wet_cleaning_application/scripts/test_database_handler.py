#!/usr/bin/env python

import unittest
import database_utils
import rospkg

from database_handler import DatabaseHandler as DH
from database import Database
from datetime import datetime

PKG = 'baker_wet_cleaning_application'
NAME = 'database_handler_test'
TRASH_TASK = -1
DRY_TASK = 0
WET_TASK = 1
BOTH_TASKS = 2


class TestDatabaseHandler(unittest.TestCase):

    @staticmethod
    def getDatabaseLocation():
        rospack = rospkg.RosPack()
        return str(rospack.get_path('baker_wet_cleaning_application') + '/resources')

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
        self.assertTrue(DH.isDryCleaningMethod(2))
        self.assertTrue(DH.isDryCleaningMethod(0))
        self.assertFalse(DH.isDryCleaningMethod(1))

    def testIsWetDay(self):
        self.assertTrue(DH.isWetCleaningMethod(2))
        self.assertTrue(DH.isWetCleaningMethod(1))
        self.assertFalse(DH.isWetCleaningMethod(0))

    def testIsTrashCleaningMethod(self):
        self.assertTrue(DH.isTrashCleaningMethod(0))
        self.assertTrue(DH.isTrashCleaningMethod(1))
        self.assertTrue(DH.isTrashCleaningMethod(2))

    def applyScenarios(self, scenarios, expected_outputs, evaluator):
        database_location = self.getDatabaseLocation()
        for k in range(len(scenarios)):
            database_utils.updateDatabaseToScenario(scenarios[k], database_location=database_location + '/json/')
            database = Database(extracted_file_path=database_location)
            database_handler = DH(database)
            computed_output = evaluator(database_handler)
            self.assertEqual(computed_output, expected_outputs[k],
                             msg='Test index {} failed computed {} != expected {}'.format(k, computed_output, expected_outputs))

    @staticmethod
    def dueRoomsEvaluator(database_handler):
        database_handler.computeAllDueRooms()
        due_rooms = database_handler.due_rooms_
        due_rooms_ids = set([room.room_id_ for room in due_rooms])
        return due_rooms_ids

    def testRestoreDueRooms(self):
        scenarios = [{},
                     {'open_cleaning_tasks': {}},
                     {'open_cleaning_tasks': {
                         '1': [TRASH_TASK, DRY_TASK],
                         '8': [WET_TASK, TRASH_TASK],
                         '4': [TRASH_TASK]
                     }},
                     {'open_cleaning_tasks': {
                         '1': [],
                         '10': [WET_TASK]
                     }}]

        expected_outputs = [set([]), set([]), {4, 1, 8}, {10}]
        self.applyScenarios(scenarios=scenarios, expected_outputs=expected_outputs, evaluator=self.dueRoomsEvaluator)

    def testComputeAllDueRoomsEarlierRun(self):
        def evaluator(database_handler):
            return database_handler.computeAllDueRooms()

        scenarios = [{},
                     {
                         'open_cleaning_tasks': {'5': [TRASH_TASK]},
                         'last_planning_date': ['TODAY', 'TODAY']
                     },
                     {
                         'open_cleaning_tasks': {'3': [WET_TASK]},
                         'last_planning_date': ['YESTERDAY', 'YESTERDAY']
                     }]
        expected_outputs = [True, False, True]
        self.applyScenarios(scenarios=scenarios, expected_outputs=expected_outputs, evaluator=evaluator)

    def testComputeAllDueRooms(self):

        scenarios = [{
            'cleaning_methods': {'5': TRASH_TASK, '4': DRY_TASK, '7': BOTH_TASKS, '8': WET_TASK}
        }]
        # todo (rmb-ma) be more accurate on the output: testing tasks in each room and not only the rooms
        expected_outputs = [{5, 4, 8, 7}]

        self.applyScenarios(scenarios=scenarios, expected_outputs=expected_outputs, evaluator=self.dueRoomsEvaluator)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestDatabaseHandler)
