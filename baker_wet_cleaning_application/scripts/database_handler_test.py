#!/usr/bin/env python

# todo: see how using tabs for identation

PKG = 'baker_wet_cleaning_application'
NAME = 'database_handler_test'

import unittest
import rospkg

from database_handler import DatabaseHandler as DH
from database import Database

from dry_cleaning_behavior import DryCleaningBehavior

class TestDatabaseHandler(unittest.TestCase):

    database_ = None

    @classmethod
    def setUpClass(cls):
        # Loading database
        rospack = rospkg.RosPack()
        print('rospack_path' + rospack.get_path('baker_wet_cleaning_application'))
        cls.database_ = Database(
            extracted_file_path=str(rospack.get_path('baker_wet_cleaning_application') + '/resources_test'))
        cls.database_.loadDatabase()

        cls.database_handler_ = DH(cls.database_)

    def setUp(self): # todo (rmb-ma): remove?
        print "[setUp] Hello world !"

    def tearDown(self):
        # Reset the database
        print "[tearDown] Hello world"

    @classmethod
    def tearDownClass(cls): # todo (rmb-ma): remove
        print "[tearDownClass] Hello world! "

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

    def testGetAllDueRooms(self):
        print('[getAllDueRooms] TODO')
        # No earlier run
        #l
        self.assertFalse(True)

    # todo (rmb-ma) split it into a new file?
    def testDryCleaningBehavior(self):
        pass


if __name__ == '__main__':
    import rostest
    print("hello world")
    rostest.rosrun(PKG, NAME, TestDatabaseHandler)
