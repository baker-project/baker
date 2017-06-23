#!/usr/bin/env python

import actionlib
import numpy
import roslib
import rospy
import tf

#move_base_msgs
from ipa_building_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose2D, Point32
from sensor_msgs.msg import Image
from move_base_msgs.msg import *
from scitos_msgs.msg import MoveBasePathAction
from scitos_msgs.msg import MoveBasePathGoal

#import simple_script_server
#sss = simple_script_server.simple_script_server()

#class SeqControl(simple_script_server.script):
class SeqControl():
	def explore(self):
		rospy.init_node('exploration_node')

		# subscribe to map topic
		rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
		self.map_received = False
		while self.map_received == False:
			rospy.sleep(0.5)

		# compute exploration path
		#planning_mode = 2 # viewpoint planning
		#fov_points = [Point32(x=0.15, y=0.35), Point32(x=0.15, y=-0.35), Point32(x=1.15, y=-0.65), Point32(x=1.15, y=0.65)] # this field of view fits a Asus Xtion sensor mounted at 0.63m height (camera center) pointing downwards to the ground in a respective angle
		# ... or ...
		planning_mode = 1 # footprint planning
		fov_points = [Point32(x=-0.3, y=0.3), Point32(x=-0.3, y=-0.3), Point32(x=0.3, y=-0.3), Point32(x=0.3, y=0.3)] # this is the working area of a vacuum cleaner with 60 cm width

		exploration_goal = RoomExplorationGoal()
		exploration_goal.input_map = self.map;
		exploration_goal.map_resolution = self.map_resolution
		exploration_goal.map_origin = self.map_origin
		exploration_goal.robot_radius = 0.3
		exploration_goal.coverage_radius = 0.3
		exploration_goal.field_of_view = fov_points
		exploration_goal.starting_position = Pose2D(x=0., y=0., theta=0.)
		exploration_goal.planning_mode = planning_mode
		
		exploration_client = actionlib.SimpleActionClient('/room_exploration/room_exploration_server', RoomExplorationAction)
		exploration_client.wait_for_server()
		exploration_client.send_goal(exploration_goal)
		exploration_client.wait_for_result()
		print exploration_client.get_result()

		# command robot movement
	
	def mapCallback(self, msg):
		# convert to image message
		print ("Reading map ...")
		# read the data from msg.data and convert it from [0,100] to [0,255], furthermore, set the image properties
		self.map = Image()
		self.map.header = msg.header
		self.map.height = msg.info.height
		self.map.width = msg.info.width
		self.map.encoding = "mono8"
		self.map.step = msg.info.width
		self.map.data = []
		for value in msg.data:
			self.map.data.append(numpy.uint8(255./100. * value)) # todo: this is buggy and lets the receiving program crash on applying cv_bridge

		self.map_resolution = msg.info.resolution
		self.map_origin = Pose2D(x=msg.info.origin.position.x, y=msg.info.origin.position.y, theta=0.)

		self.map_received = True
		print("Map received.")

if __name__ == '__main__':
	try:
		script = SeqControl()
		script.explore()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
