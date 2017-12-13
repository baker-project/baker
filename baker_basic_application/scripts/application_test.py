#!/usr/bin/env python

import application_container
import behavior_container
import application_test_interruptor
import rospy
import thread


class TestBehavior1(behavior_container.BehaviorContainer):

	def returnToRobotStandardState(self):
		print "Behavior 1: Returning to standard state"

	def executeCustomBehavior(self):
		print "Behavior 1: Executing..."
		i = 0
		while(i < 1E5):
			i = i + 1
			print "Behavior 1: %i" % i
			if (handle_interrupt != 0):
				return
		print "Behavior 1: Execution completed."


class CppBehavior(behavior_container.BehaviorContainer):

	def returnToRobotStandardState(self):
		print "C++ Sub Behavior: Returning to standard state"

	def executeCustomBehavior(self):
		print "C++ Sub Behavior: Executing..."
		segmentation_goal = MapSegmentationGoal()
		segmentation_goal.input_map = self.map_data.map  # todo: use the current room map
		segmentation_goal.map_resolution = self.map_data.map_resolution
		segmentation_goal.map_origin = self.map_data.map_origin
		segmentation_goal.return_format_in_meter = False
		segmentation_goal.return_format_in_pixel = True
		segmentation_goal.robot_radius = 0.3
		print "Waiting for Action '/room_segmentation/room_segmentation_server' to become available..."
		segmentation_client = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server', MapSegmentationAction)
		runAction(segmentation_client, segmentation_goal)
		print "C++ Sub Behavior: Completed"
		

class TestBehavior2(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var):
		# Run the constructor of the superclass
		super(BehaviorContainer, self).__init__(interrupt_var)
		# Append a new C++ Behavior instance to the sub behaviors
		self.sub_behaviors.append(CppBehavior(interrupt_var))

	def returnToRobotStandardState(self):
		print "Behavior 2: Returning to standard state"

	def executeCustomBehavior(self):
		print "Behavior 2: Executing..."
		# move to the room center of the current room
		sub_behaviors[0].executeBehavior()
		print "Behavior 2: Execution completed."



class TestApplication(application_container.ApplicationContainer):

	def pauseProcedure(self):
		print "Application paused"

	def cancelProcedure(self):
		print "Application cancelled"

	def returnToRobotStandardState(self):
		print "Application: returning to Robot Standard State"

	def executeCustomBehavior(self):
		print "Beginning execution"
		behavior_1 = TestBehavior1(self.application_status)
		behavior_2 = TestBehavior2(self.application_status)
		print "Executing behavior 1"
		behavior_1.executeCustomBehavior()
		self.handle_interrupt()
		print "Executing behavior 2"
		behavior_2.executeCustomBehavior()
		self.handle_interrupt()
		print "Execution completed"
		

	
if __name__ == '__main__':
	try:
		# Initialize and start interruptor
		#application_test_interruptor.initialize()
		#thread.start_new_thread(application_test_interruptor.interrupt, ())
		# Initialize and start test application
		app = TestApplication("interrupt_test_app")
		#app.executeCustomBehavior()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"