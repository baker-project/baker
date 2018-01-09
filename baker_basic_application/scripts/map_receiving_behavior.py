#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from baker_msgs.srv import *

import behavior_container

class MapReceivingBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no current data to save
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		# receive the navigation map in sensor_msgs/Image format
		print "Waiting for service " + str(self.service_str) + " to become available ..."
		rospy.wait_for_service(str(self.service_str))
		try:
			get_map = rospy.ServiceProxy(str(self.service_str), GetMap)
			self.map_data = get_map()
		except rospy.ServiceException, e:
			print "Map-Receiving-Service call failed: %s" % e
			self.behavior_status = 2
		print "Map received with resolution: ", self.map_data.map_resolution, "   and origin: ", self.map_data.map_origin