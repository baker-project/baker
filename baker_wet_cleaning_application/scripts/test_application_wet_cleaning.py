#!/usr/bin/env python

from test_database_handler import TestDatabaseHandler

PKG = 'baker_wet_cleaning_application'
import roslib
roslib.load_manifest(PKG)

# =========================================================
# Highest element in the hierarchy of the testing module
# Launch and start all the tests (unittests and integration test)

if __name__ == '__main__':
	import rostest
	rostest.rosrun(PKG, 'test_database_handler', TestDatabaseHandler)
