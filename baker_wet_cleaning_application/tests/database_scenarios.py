#!/usr/bin/env python

import argparse
import json
from datetime import datetime, timedelta

ROOM_NUMBER = 11
DATABASE_LOCATION = '../resources/json/'
DATE_FORMAT = '%Y-%m-%d_%H:%M'


def cleaningMethod(string):
	method = int(string)
	if method < -1 or method > 2:
		msg = "{} is not a valid cleaning method (-1: nothing, 0: dry, 1: wet, 2: both)".format(method)
		raise argparse.ArgumentTypeError(msg)
	return method


def updateRooms(methods, reset_opened_tasks=False, reset_timestamps=False):
	rooms_filename = DATABASE_LOCATION + 'rooms.json'
	with open(rooms_filename, 'r') as rf:
		data = json.load(rf)

	today = datetime.now()
	week_type = today.isocalendar()[1] % 2
	week_day = today.weekday()
	day_index = week_type * 7 + week_day

	previous_date = today - timedelta(days=3)

	keys = [str(k) for k in data.keys()]
	for key in keys:
		if reset_opened_tasks:
			data[key]['open_cleaning_tasks'] = []

		if reset_timestamps:

			data[key]['room_cleaning_datestamps'] = [previous_date.strftime(DATE_FORMAT)] * 3

		# todo (rmb-ma). doesn't work if ids are not following
		data[key]['room_cleaning_method'] = methods[int(key)]

		data[key]['room_scheduled_days'][day_index] = 'x' if methods[int(key)] > 0 else ''

	with open(rooms_filename, 'w') as wf:
		json.dump(data, wf, indent=4, sort_keys=True)

def resetLastPlanningDate():
	app_filename = DATABASE_LOCATION + 'application_data.json'
	with open(app_filename, 'r') as rf:
		data = json.load(rf)

	today = datetime.now()
	new_date = today - timedelta(days=1)
	data['last_planning_date'] = [new_date.strftime(DATE_FORMAT)]*2

	with open(app_filename, 'w') as wf:
		json.dump(data, wf, indent=4, sort_keys=True)

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Reset the database to a given scenario')

	for k in range(ROOM_NUMBER):
		parser.add_argument('-r{}'.format(k), '--room{}'.format(k), type=cleaningMethod, nargs='?', default=-1,
							help='Cleaning method of room {} (-1: nothing, 0: dry, 1: wet, 2: both). Default -1'.format(k))
	args = vars(parser.parse_args())

	cleaning_methods = [args.get('room{}'.format(k)) for k in range(ROOM_NUMBER)]
	updateRooms(cleaning_methods, reset_opened_tasks=True, reset_timestamps=True)
	resetLastPlanningDate()
