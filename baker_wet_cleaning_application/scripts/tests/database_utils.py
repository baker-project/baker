#!/usr/bin/env python

import json
from datetime import datetime, timedelta

# =========================================================
# Contains some functions to manipulate the json database
# =========================================================

DATE_FORMAT = '%Y-%m-%d_%H:%M'
DATABASE_LOCATION = '../resources/json/'

def loadRoomsDatabase(filename):
	f = open(filename, 'r')
	return json.load(f)


def saveRoomsDatabase(filename, data):
	f = open(filename, 'w')
	json.dump(f, indent=4, sort_keys=True)


def reset(data, reset_opened_tasks=False, reset_timestamps=False):
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

	return data

def updateRooms(data, methods, reset_opened_tasks=False, reset_timestamps=False):

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

	return data


def updateDatabaseToScenario(scenario):
	data = loadRoomsDatabase(DATABASE_LOCATION + 'rooms.json')
	for key in [str(k) for k in data.keys()]:

		data[key]['open_cleaning_tasks'] = []
		if 'open_cleaning_tasks' in scenario.keys and key in scenario['open_cleaning_tasks'].keys():
			data[key]['open_cleaning_tasks'] = scenario['open_cleaning_tasks'][key]

	saveRoomsDatabase(DATABASE_LOCATION + 'rooms.json', data)