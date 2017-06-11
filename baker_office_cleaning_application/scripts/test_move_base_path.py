#!/usr/bin/env python

import math

import actionlib
import roslib
import rospy
import tf

#move_base_msgs
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
from scitos_msgs.msg import MoveBasePathAction
from scitos_msgs.msg import MoveBasePathGoal

def simple_move():
	print("Starting move test")
	rospy.init_node('move_node')
	sac = actionlib.SimpleActionClient('move_base_path', MoveBasePathAction )
	goal1 = MoveBasePathGoal()
	#goal2 = MoveBaseGoal()

	poses = [[-1.,-3.5],
		 [-0.99,-3.5],
		 [1.,-3.5],
		 [1.,-3.51],
		 [1.,-5.5],
		 [0.99,-5.5],
		 [-1.,-5.5],
		 [-1.,-5.49]
		]

	#set goal
	for i in range(0, 80):
		goal_pose = PoseStamped()
		#goal_pose.header.frame_id = 'map'
		#goal_pose.header.stamp = rospy.Time.now()
		goal_pose.pose.position.x = 0.4*i
		if i>31 and i<42:
			goal_pose.pose.position.x = 12.4
		if i>=42:
			goal_pose.pose.position.x = 12.4 - 0.4*(i-42)
		goal_pose.pose.position.y = 0.7*math.sin(i*0.15)
		#goal_pose.pose.position.x = poses[i%8][0]
		#goal_pose.pose.position.y = poses[i%8][1]
		angle = 0
		if i!=0:
			angle = math.atan2(goal_pose.pose.position.y-goal1.target_poses[-1].pose.position.y, goal_pose.pose.position.x-goal1.target_poses[-1].pose.position.x)
		quaternion = tf.transformations.quaternion_from_euler(angle, 0., 0., 'rzyx')#1.57)
		goal_pose.pose.orientation.x = quaternion[0]
		goal_pose.pose.orientation.y = quaternion[1]
		goal_pose.pose.orientation.z = quaternion[2]
		goal_pose.pose.orientation.w = quaternion[3]
		#goal_pose.header
		goal1.target_poses.append(goal_pose)
	goal1.path_tolerance = 0.1
	goal1.goal_position_tolerance = 0.5
	goal1.goal_angle_tolerance = 0.085
	print("Waiting for server")
	sac.wait_for_server()
	print("Sending command")
	sac.send_goal(goal1)
	print("Waiting for result")
	sac.wait_for_result()
	print sac.get_result()


if __name__ == '__main__':
	try:
		simple_move()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
