#!/usr/bin/env python

from database_handler_test import TestDatabaseHandler

# =========================================================
# Highest element in the hierarchy of the testing module
# Launch and start all the tests (unittests and integration test)

PKG = 'baker_wet_cleaning_application'

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'test_database_handler', TestDatabaseHandler)
