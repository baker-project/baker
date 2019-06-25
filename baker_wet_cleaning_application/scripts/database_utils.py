#!/usr/bin/env python

import json
from datetime import datetime, timedelta
import os

# =========================================================
# Contains some functions to manipulate the json database
# =========================================================

DATE_FORMAT = '%Y-%m-%d_%H:%M'
DATABASE_LOCATION = '../resources/json'
OFFSET = 24*8*60 # todo (rmb-ma) read from database


def updateAndReturnPreviousPlanningOffset(planning_offset, database_location=DATABASE_LOCATION):
	filename = database_location + '/application_data.json'
	data = loadJsonDatabase(filename)
	previous_planning_offset = data['planning_offset']
	data['planning_offset'] = planning_offset
	saveJsonDatabase(filename, data)
	return previous_planning_offset

def getRobotTodayIndex(offset):
	today = datetime.now() - timedelta(minutes=offset)
	week_type = today.isocalendar()[1] % 2
	week_day = today.weekday()
	today_index = week_type * 7 + week_day
	return today_index

def loadJsonDatabase(filename):
	f = open(filename, 'r')
	return json.load(f)


def saveJsonDatabase(filename, data):
	f = open(filename, 'w')
	json.dump(data, f, indent=4, sort_keys=True)


def reset(data, reset_opened_tasks=False, reset_timestamps=False, reset_scheduled_tasks=False, reset_tmp_database=False):
	if reset_tmp_database:
		removeTmpDatabase(DATABASE_LOCATION)
	previous_date = datetime.now() - timedelta(days=3)

	keys = [str(k) for k in data.keys()]
	for key in keys:
		if reset_opened_tasks:
			data[key]['open_cleaning_tasks'] = []

		if reset_timestamps:
			data[key]['room_cleaning_datestamps'] = [previous_date.strftime(DATE_FORMAT)] * 3

		if reset_scheduled_tasks:
			data[key]['room_scheduled_days'] = ['']*14
	return data


def getRoomIds(database_location=DATABASE_LOCATION):
	rooms_data = loadJsonDatabase(database_location + 'rooms.json')
	return [str(key) for key in rooms_data.keys()]


def readOffset(database_location):
	data = loadJsonDatabase(database_location + 'application_data.json')
	if data['planning_offset'] is None:
		data['planning_offset'] = 0
		saveJsonDatabase(database_location - 'application_data.json', data)
	return data['planning_offset']


def updateRooms(data, methods, offset=0, reset_opened_tasks=False, reset_timestamps=False, reset_tmp_database=False):
	if reset_tmp_database:
		removeTmpDatabase(database_location=DATABASE_LOCATION)

	today_index = getRobotTodayIndex(offset)

	previous_date = datetime.now() - timedelta(days=3)

	keys = [str(k) for k in data.keys()]
	for key in keys:
		if reset_opened_tasks:
			data[key]['open_cleaning_tasks'] = []

		if reset_timestamps:
			data[key]['room_cleaning_datestamps'] = [previous_date.strftime(DATE_FORMAT)] * 3

		# todo (rmb-ma). doesn't work if ids are not following
		data[key]['room_cleaning_method'] = methods[key]

		data[key]['room_scheduled_days'] = ['']*14
		if methods[key] >= 0:
			print('key {} ; today index {}'.format(key, today_index))
		data[key]['room_scheduled_days'][today_index] = 'x' if methods[key] >= 0 else ''

	return data

def removeTmpDatabase(database_location):
	filenames = ['tmp_application_data.json', 'tmp_rooms.json']
	for filename in filenames:
		try:
			os.remove(database_location + '/' + filename)
		except OSError as error:
			print(error)
			pass


def updateDatabaseToScenario(scenario, database_location=DATABASE_LOCATION, planning_offset=0):
	removeTmpDatabase(database_location)
	rooms_data = loadJsonDatabase(database_location + 'rooms.json')
	rooms_data = reset(rooms_data, reset_opened_tasks=True, reset_timestamps=True, reset_scheduled_tasks=True)

	today = datetime.now() - timedelta(minutes=planning_offset)
	week_type = today.isocalendar()[1] % 2
	week_day = today.weekday()
	today_index = week_type * 7 + week_day

	for room_id in [str(k) for k in rooms_data.keys()]:

		if 'open_cleaning_tasks' in scenario.keys() and room_id in scenario['open_cleaning_tasks'].keys():
			rooms_data[room_id]['open_cleaning_tasks'] = scenario['open_cleaning_tasks'][room_id]

		if 'cleaning_methods' in scenario.keys() and room_id in scenario['cleaning_methods'].keys():
			method = scenario['cleaning_methods'][room_id]
			rooms_data[room_id]['room_cleaning_method'] = method if method != -1 else 2

			if 'room_scheduled_days' not in scenario.keys() or room_id not in scenario['room_scheduled_days'].keys():
				rooms_data[room_id]['room_scheduled_days'][today_index] = 'x' if method != -1 else 'p'

		if 'room_cleaning_datestamps' in scenario.keys() and room_id in scenario['room_cleaning_datestamps'].keys():
			for method_index in range(3):
				# days_delta_str == 'D-5' means 5 days before now (now == 'D-0')
				days_delta_str = scenario['room_cleaning_datestamps'][room_id][method_index]
				days_delta = int(days_delta_str[1:])
				rooms_data[room_id]['room_cleaning_datestamps'][method_index] = (today + timedelta(days=days_delta)).strftime(DATE_FORMAT)

		if 'room_scheduled_days' in scenario.keys() and room_id in scenario['room_scheduled_days'].keys():
			for (days_delta_str, method) in scenario['room_scheduled_days'][room_id].items():
				days_delta = int(days_delta_str[1:])
				date_index = (today_index + days_delta) % 14
				rooms_data[room_id]['room_scheduled_days'][date_index] = method

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