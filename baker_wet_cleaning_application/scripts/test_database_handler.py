#!/usr/bin/env python

import unittest
import database_utils
import rospkg

from database_handler import DatabaseHandler as DH
from database import Database
from random import randint

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

    def applyScenarios(self, scenarios, expected_outputs, evaluator, planning_offset=0):
        database_location = self.getDatabaseLocation()
        for k in range(len(scenarios)):
            previous_planning_offset = database_utils.updateAndReturnPreviousPlanningOffset(planning_offset,
                                                                                            database_location=database_location + '/json/')
            database_utils.updateDatabaseToScenario(scenarios[k], database_location=database_location + '/json/', planning_offset=planning_offset)
            database = Database(extracted_file_path=database_location)
            database_handler = DH(database)
            computed_output = evaluator(database_handler)
            database_utils.updateAndReturnPreviousPlanningOffset(previous_planning_offset, database_location=database_location + '/json/')
            self.assertEqual(computed_output, expected_outputs[k],
                             msg='Test index {} failed computed {} != expected {}'.format(k, computed_output, expected_outputs[k]))

    @staticmethod
    def dueRoomsEvaluator(database_handler):
        database_handler.computeAllDueRooms()
        due_rooms = database_handler.due_rooms_
        cleaning_tasks = {}
        for room in due_rooms:
            cleaning_tasks[str(room.room_id_)] = set(room.open_cleaning_tasks_)
        return cleaning_tasks

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
        expected_outputs = [{},
                            {},
                            {
                                '4': {TRASH_TASK},
                                '8': {WET_TASK, TRASH_TASK},
                                '1': {TRASH_TASK, DRY_TASK}
                            },
                            {
                                '10': {WET_TASK}
                            }]
        self.applyScenarios(scenarios=scenarios, expected_outputs=expected_outputs,
                            evaluator=self.dueRoomsEvaluator, planning_offset=randint(0, 3000))

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
        self.applyScenarios(scenarios=scenarios, expected_outputs=expected_outputs,
                            evaluator=evaluator, planning_offset=randint(0, 3000))

    def testComputeAllDueRooms(self):
        scenarios = [{
            'cleaning_methods': {'5': TRASH_TASK, '4': DRY_TASK, '7': BOTH_TASKS, '8': WET_TASK}
        }]

        expected_outputs = [{
            '5': {TRASH_TASK},
            '4': {DRY_TASK, TRASH_TASK},
            '8': {WET_TASK, TRASH_TASK},
            '7': {DRY_TASK, TRASH_TASK, WET_TASK}
        }]

        self.applyScenarios(scenarios=scenarios, expected_outputs=expected_outputs,
                            evaluator=self.dueRoomsEvaluator, planning_offset=randint(0, 3000))

    @staticmethod
    def overdueRoomsEvaluator(database_handler):
        database_handler.computeAllDueRooms()  # todo (rmb-ma) check if it should be called in computeAllOverdueRooms
        database_handler.computeAllOverdueRooms()
        overdue_rooms = database_handler.overdue_rooms_
        cleaning_tasks = {}
        for room in overdue_rooms:
            cleaning_tasks[str(room.room_id_)] = set(room.open_cleaning_tasks_)
        return cleaning_tasks

    def testComputeAllOverdueRooms(self):
        scenarios = [{
            'room_cleaning_datestamps': {
                '1': ['D-8', 'D-7', 'D-10'],  # [TRASH_DATE, DRY_DATE, WET_DATE]
                '10': ['D-8', 'D-9', 'D-12']
            },
            'room_scheduled_days': {
                '1': {'D-3': 'x', 'D-4': 'p'},
                '10': {'D-5': 'p'},
                '7': {'D-1': 'x'}
            },
            'cleaning_methods': {'1': DRY_TASK, '10': BOTH_TASKS, '7': DRY_TASK}
        }]
        expected_outputs = [{
            '1': {TRASH_TASK, DRY_TASK},
            '10': {TRASH_TASK},
            '7': {DRY_TASK, TRASH_TASK}
        }]

        self.applyScenarios(scenarios=scenarios, expected_outputs=expected_outputs,
                            evaluator=self.overdueRoomsEvaluator, planning_offset=randint(0, 3000))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestDatabaseHandler)
