#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from baker_msgs.srv import *

import behavior_container

class MapReceivingBehavior(behavior_container.BehaviorContainer):

	def __init__(self, behavior_name, interrupt_var, service_str, map_segmented_receiving_service_str=None):
		self.behavior_name_ = behavior_name
		self.interrupt_var_ = interrupt_var
		self.service_str_ = service_str
		self.map_segmented_receiving_service_str_ = map_segmented_receiving_service_str
	
	# Method for setting parameters for the behavior
	def setParameters(self):
		# no parameters required
		pass

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no current data to save
		# nothing to be undone
		pass

	# Implemented Behavior
	def executeCustomBehavior(self):
		# receive the navigation map in sensor_msgs/Image format
		self.printMsg("Waiting for service " + self.service_str_ + " to become available ...")
		rospy.wait_for_service(self.service_str_)
		try:
			get_map = rospy.ServiceProxy(self.service_str_, baker_msgs.srv.GetMap)
			self.map_data_ = get_map()
		except rospy.ServiceException, e:
			print "Map-Receiving-Service call failed: %s" % e
			self.behavior_status_ = 2		# todo: what does this do?
		print "Map received with resolution: ", self.map_data_.map_resolution, "   and origin: ", self.map_data_.map_origin
		
		# optionally receive the segmented map in sensor_msgs/Image format
		if (self.map_segmented_receiving_service_str_ != None):
			self.printMsg("Waiting for service " + str(self.map_segmented_receiving_service_str_) + " to become available ...")
			try:
				rospy.wait_for_service(self.map_segmented_receiving_service_str_, timeout=3.0)
				get_map_segmented = rospy.ServiceProxy(self.map_segmented_receiving_service_str_, baker_msgs.srv.GetMap)
				self.map_segmented_data_ = get_map_segmented()
			except rospy.ServiceException, e:
				print "No segmented map available: %s" % e
				self.map_segmented_data_ = None
				self.behavior_status_ = 2
			print "Segmented map received with resolution: ", self.map_segmented_data_.map_resolution, "   and origin: ", self.map_segmented_data_.map_origin
