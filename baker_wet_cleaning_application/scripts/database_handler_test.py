#!/usr/bin/env python

# todo: see how using tabs for identation

import unittest

from database_handler import DatabaseHandler as DH

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



if __name__ == '__main__':
    import rostest
    rostest.rosrun('baker_wet_cleaning_application', 'test_database_handler', TestDatabaseHandler)
    unittest.main()