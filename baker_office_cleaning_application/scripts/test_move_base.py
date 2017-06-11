#!/usr/bin/env python

import actionlib
import roslib
import rospy


#move_base_msgs
from move_base_msgs.msg import *

def simple_move():
	print("Starting move test")
	rospy.init_node('move_node')
	sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
	goal = MoveBaseGoal()
	
	#set goal
	goal.target_pose.header.frame_id = 'base_link'
	goal.target_pose.pose.position.x = 0.  #-1.0
	goal.target_pose.pose.position.y = 0.  #-3.5
	goal.target_pose.pose.orientation.w = 0.0
	goal.target_pose.header.stamp = rospy.Time.now()
	print("Waiting for server")
	sac.wait_for_server()
	print("Sending command")
	sac.send_goal(goal)
	print("Waiting for result")
	sac.wait_for_result()
	print sac.get_result()


if __name__ == '__main__':
	try:
		simple_move()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
