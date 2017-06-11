#!/usr/bin/env python

import math

import actionlib
import roslib
import rospy
import tf

from geometry_msgs.msg import PoseStamped
#move_base_msgs
from move_base_msgs.msg import *
from scitos_msgs.msg import MoveBasePathAction
from scitos_msgs.msg import MoveBasePathGoal


def simple_move():
	print("Starting move test")
	rospy.init_node('move_node')
	sac = actionlib.SimpleActionClient('move_base_path', MoveBasePathAction )
	print("Waiting for server")
	sac.wait_for_server()

	ranges = [[-1, 1, -3.5, -3.5],
		  [1, 1, -3.5, -5.5],
		  [1, -1, -5.5, -5.5],
		  [-1, -1, -5.5, -3.5]]

	for k in range(0, 4):
		#set goal
		goal = MoveBasePathGoal()
		waypoints = 50
		for i in range(0, waypoints):
			goal_pose = PoseStamped()
			#goal_pose.header.frame_id = 'map'
			#goal_pose.header.stamp = rospy.Time.now()
			#goal_pose.pose.position.x = 0.3*i
			#goal_pose.pose.position.y = 0.5*math.sin(i*0.15)
			goal_pose.pose.position.x = ranges[k][0] + float(i)/waypoints*(ranges[k][1]-ranges[k][0])
			goal_pose.pose.position.y = ranges[k][2] + float(i)/waypoints*(ranges[k][3]-ranges[k][2])
			angle = 0
			if i!=0:
				angle = math.atan2(goal_pose.pose.position.y-goal.target_poses[-1].pose.position.y, goal_pose.pose.position.x-goal.target_poses[-1].pose.position.x)
			quaternion = tf.transformations.quaternion_from_euler(angle, 0., 0., 'rzyx')#1.57)
			goal_pose.pose.orientation.x = quaternion[0]
			goal_pose.pose.orientation.y = quaternion[1]
			goal_pose.pose.orientation.z = quaternion[2]
			goal_pose.pose.orientation.w = quaternion[3]
			if i==1:
				goal.target_poses[0].pose.orientation = goal_pose.pose.orientation
			goal.target_poses.append(goal_pose)
		goal.path_tolerance = 0.05
		goal.goal_position_tolerance = 0.5
		goal.goal_angle_tolerance = 0.085
		print("Sending command")
		sac.send_goal(goal)
		print("Waiting for result")
		sac.wait_for_result()
		print "Finished: ", sac.get_result()



if __name__ == '__main__':
	try:
		simple_move()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
