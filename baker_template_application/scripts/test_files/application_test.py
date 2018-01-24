#!/usr/bin/env python

import application_container
import behavior_container
import application_test_interruptor
import rospy
import threading
from ipa_building_msgs.msg import *

import roslib
roslib.load_manifest('baker_wet_cleaning_application')
import actionlib
import tf
import numpy as np
import cv2
import sys
import time

from baker_msgs.srv import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose2D, Point32, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TestBehavior1(behavior_container.BehaviorContainer):

	# This test behavior class increments an integer from 0 to a certain max number stated below.

	max_count = 100

	def returnToRobotStandardState(self):
		self.printMsg("Returning to standard state")

	def executeCustomBehavior(self):
		self.printMsg("Executing...")
		i = 0
		while(i < max_count):
			i = i + 1
			rospy.sleep(0.5)
			self.printMsg(str(i))
			if (self.handleInterrupt() != 0):
				return
		self.printMsg("Execution completed.")


class CppBehavior(behavior_container.BehaviorContainer):

	# This test behavior class calls services written in C++. Those services / action servers are
	# - get map image
	# - room segmentation

	def returnToRobotStandardState(self):
		self.printMsg("Returning to standard state")

	def executeCustomBehavior(self):
		self.printMsg("Executing...")

		# receive the navigation map in sensor_msgs/Image format
		self.printMsg("Waiting for service '/baker/get_map_image' to become available ...")
		rospy.wait_for_service('/baker/get_map_image')
		try:
			get_map = rospy.ServiceProxy('/baker/get_map_image', GetMap)
			self.map_data = get_map()
		except rospy.ServiceException, e:
			print "Map-Receiving-Service call failed: %s" % e
		print "Map received with resolution: ", self.map_data.map_resolution, "   and origin: ", self.map_data.map_origin

		# compute map division into rooms (ipa_room_segmentation)
		segmentation_goal = MapSegmentationGoal()
		segmentation_goal.input_map = self.map_data.map  
		# rospy.init_node('exploration_node')# todo: use the current room map
		segmentation_goal.map_resolution = self.map_data.map_resolution
		segmentation_goal.map_origin = self.map_data.map_origin
		segmentation_goal.return_format_in_meter = False
		segmentation_goal.return_format_in_pixel = True
		segmentation_goal.robot_radius = 0.3
		segmentation_client = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server', MapSegmentationAction)
		self.printMsg("Running Action /room_segmentation/room_segmentation_server ...")
		segmentation_result = self.runAction(segmentation_client, segmentation_goal)
		self.printMsg("Segmentation completed")

		self.printMsg("Completed")
		

class TestBehavior2(behavior_container.BehaviorContainer):

	# This test behavior class calls contains a CppBehavior object and runs it

	def __init__(self, interrupt_var):
		self.sub_behavior = CppBehavior(interrupt_var)

	def returnToRobotStandardState(self):
		self.printMsg("Returning to standard state")

	def executeCustomBehavior(self):
		self.printMsg("Executing...")
		self.sub_behavior.executeBehavior()
		self.printMsg("Execution completed.")



class TestApplication(application_container.ApplicationContainer):

	# This application first runs an TestBehavior1 instance, then an TestBehavior2 instance.
	# The TestBehavor2 instance runs a contained CppBehavior instance.

	def prePauseProcedure(self):
		print "Application paused"

	def postPauseProcedure(self):
		print "Application: Application continued."

	def cancelProcedure(self):
		print "Application cancelled"

	def returnToRobotStandardState(self):
		print "Application: returning to Robot Standard State"

	def executeCustomBehavior(self):
		print "Beginning execution"
		behavior_1 = TestBehavior1(self.application_status)
		behavior_1.behavior_name = "Test 1"
		behavior_2 = TestBehavior2(self.application_status)
		behavior_2.behavior_name = "Test 2"
		print "Executing behavior 1"
		behavior_1.executeCustomBehavior()
		if (self.handleInterrupt() == 2):
			return
		print "Executing behavior 2"
		behavior_2.executeCustomBehavior()
		self.handleInterrupt()
		print "Execution completed"
		

	
if __name__ == '__main__':
	try:
		# Initialize node
		rospy.init_node('application_test_server')
		# Initialize application
		app = TestApplication("interrupt_test_app")
		# Execute application
		app.executeApplication()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"