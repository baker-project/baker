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

	poses = [[0,0],
		 [17,0],
		 [-1.5, -10]
		]

	#set goal
	for p in poses:
		goal_pose = PoseStamped()
		goal1 = subsample_pose(goal1, goal_pose, p)
		
	goal1.path_tolerance = 0.1
	goal1.goal_position_tolerance = 0.1
	goal1.goal_angle_tolerance = 0.087
	print("Waiting for server")
	sac.wait_for_server()
	print("Sending command")
	sac.send_goal(goal1)
	print("Waiting for result")
	sac.wait_for_result()
	print sac.get_result()

def subsample_pose(goal1, goal_pose, sub_target):
	for i in range(100,1,-1):
		#goal_pose.header.frame_id = 'map'
		#goal_pose.header.stamp = rospy.Time.now()
		goal_pose.pose.position.x = sub_target[0]/i
		goal_pose.pose.position.y = sub_target[1]/i
		angle = 0
		quaternion = tf.transformations.quaternion_from_euler(angle, 0., 0., 'rzyx')#1.57)
		goal_pose.pose.orientation.x = quaternion[0]
		goal_pose.pose.orientation.y = quaternion[1]
		goal_pose.pose.orientation.z = quaternion[2]
		goal_pose.pose.orientation.w = quaternion[3]
		#goal_pose.header
		goal1.target_poses.append(goal_pose)
	return goal1
		


if __name__ == '__main__':
	try:
		simple_move()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
