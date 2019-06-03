#!/usr/bin/env python

import argparse
import json
from datetime import datetime, timedelta
import sys
import database_utils

ROOM_NUMBER = 11
DATABASE_LOCATION = '../resources/json/'
SET_APPLICATION_STATUS_SERVICE = '/set_application_status_application_wet_cleaning'


def cleaningMethod(string):
	method = int(string)
	if method < -1 or method > 2:
		msg = "{} is not a valid cleaning method (-1: nothing, 0: dry, 1: wet, 2: both)".format(method)
		raise argparse.ArgumentTypeError(msg)
	return method


def resetLastPlanningDate():
	app_filename = DATABASE_LOCATION + 'application_data.json'
	with open(app_filename, 'r') as rf:
		data = json.load(rf)

	today = datetime.now()
	new_date = today - timedelta(days=1)
	data['last_planning_date'] = [new_date.strftime(database_utils.DATE_FORMAT)]*2

	with open(app_filename, 'w') as wf:
		json.dump(data, wf, indent=4, sort_keys=True)


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Reset the database to a given scenario')

	parser.add_argument("--start_application", action='store_true',
						help="If you want to start the application (must be launched and connected to mira)")
	parser.add_argument("--stop_application", action='store_true',
						help="If you want to stop the application (all other arguments are ignored)")

	for k in range(ROOM_NUMBER):
		parser.add_argument('-r{}'.format(k), '--room{}'.format(k), type=cleaningMethod, nargs='?', default=-1,
							help='Cleaning method of room {} (-1: nothing, 0: dry, 1: wet, 2: both). Default -1'.format(k))
	args = vars(parser.parse_args())

	cleaning_methods = [args.get('room{}'.format(k)) for k in range(ROOM_NUMBER)]

	if args['stop_application']:
		import subprocess
		subprocess.call(['rosservice', 'call', SET_APPLICATION_STATUS_SERVICE, '2'])
		print('WARNING - nothing done on the database')
		sys.exit(1)

	rooms_filename = DATABASE_LOCATION + 'rooms.json'
	data = database_utils.loadJsonDatabase(rooms_filename)

	data = database_utils.updateRooms(data, cleaning_methods, reset_opened_tasks=True, reset_timestamps=True)
	database_utils.saveJsonDatabase(rooms_filename, data)
	resetLastPlanningDate()

	if args['start_application']:
		import subprocess
		subprocess.call(['rosservice', 'call', SET_APPLICATION_STATUS_SERVICE, '0'])
