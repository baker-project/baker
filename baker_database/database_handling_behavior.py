#!/usr/bin/env python

import behavior_container
import database
import datetime

class DatabaseHandlingBehavior(behavior_container.BehaviorContainer):
    database_ = None

    def __init__(self, database):
        self.database_ = database

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# save current data if necessary
        self.database_.saveDatabase()
		# undo or check whether everything has been undone

	# Method for setting parameters for the behavior
	def setParameters(self):
		pass


	# Implemented Behavior
	def executeCustomBehavior(self):
		# After each command, def handleInterrupt has to be executed:
		# if self.handleInterrupt() == 2:
		#     return
        due_assignments = []
        due_rooms = []
        for assignment in self.database_.assignments_:
            # Check if the last success date within the assignment is too far in the past
            if (assignment.last_completed_clean_ + assignment.clean_interval_ < datetime.datetime.now()):
                due_assignments.append(self.database_.getAssignment(assignment.assignment_id_)
        # Check all the rooms if their last success date is too far in the past
        for assignment in due_assignments:
            for room in assignment.rooms_:
                
		


'''
How to call the implemented behavior:

xyz = behavior_template.TemplateBehavior(<interrupt_var>)
xyz.behavior_name = <behavior_name>
xyz.setParameters(<parameters>)

...

xyz.executeBehavior()
'''