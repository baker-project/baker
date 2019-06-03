#!/usr/bin/env python

import json
from datetime import datetime, timedelta

# =========================================================
# Contains some functions to manipulate the json database
# =========================================================

DATE_FORMAT = '%Y-%m-%d_%H:%M'
DATABASE_LOCATION = '../resources/json'


def loadJsonDatabase(filename):
	f = open(filename, 'r')
	return json.load(f)


def saveJsonDatabase(filename, data):
	f = open(filename, 'w')
	json.dump(data, f, indent=4, sort_keys=True)


def reset(data, reset_opened_tasks=False, reset_timestamps=False, reset_scheduled_tasks=False):
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

		if reset_scheduled_tasks:
			data[key]['room_scheduled_days'] = ['']*14
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


def updateDatabaseToScenario(scenario, database_location=DATABASE_LOCATION):
	rooms_data = loadJsonDatabase(database_location + 'rooms.json')
	rooms_data = reset(rooms_data, reset_opened_tasks=True, reset_timestamps=True, reset_scheduled_tasks=True)

	today = datetime.now()
	week_type = today.isocalendar()[1] % 2
	week_day = today.weekday()
	today_index = week_type * 7 + week_day

	for key in [str(k) for k in rooms_data.keys()]:

		if 'open_cleaning_tasks' in scenario.keys() and key in scenario['open_cleaning_tasks'].keys():
			rooms_data[key]['open_cleaning_tasks'] = scenario['open_cleaning_tasks'][key]

		if 'cleaning_methods' in scenario.keys() and key in scenario['cleaning_methods'].keys():
			method = scenario['cleaning_methods'][key]
			rooms_data[key]['room_cleaning_method'] = method if method != -1 else 2
			rooms_data[key]['room_scheduled_days'][today_index] = 'x' if method != -1 else 'p'

	saveJsonDatabase(database_location + 'rooms.json', rooms_data)

	app_data = loadJsonDatabase(database_location + 'application_data.json')
	previous_date = datetime.now() - timedelta(days=3)
	app_data['last_planning_date'] = [previous_date.strftime(DATE_FORMAT)]*2

	if 'last_planning_date' in scenario.keys():
		planning_dates = scenario['last_planning_date']
		for k in range(len(planning_dates)):
			if planning_dates[k] == 'TODAY':
				app_data['last_planning_date'][k] = datetime.now().strftime(DATE_FORMAT)
				print(app_data['last_planning_date'])
			elif planning_dates[k] == 'YESTERDAY':
				app_data['last_planning_date'][k] = (datetime.now() - timedelta(days=1)).strftime(DATE_FORMAT)
	saveJsonDatabase(database_location + 'application_data.json', app_data)