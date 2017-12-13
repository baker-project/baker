#!/usr/bin/env python

import rospy
import actionlib
import application_container
import behavior_container

def initialize():
    # Create interruption client
    interrupt_goal = application_container.InterruptActionGoal()
    interrupt_goal.new_interrupt_state = 0
    print "Waiting for Action InterruptAction to become available..."
    interrupt_client = actionlib.SimpleActionClient("/baker/baker_basic_application/", application_container.InterruptActionAction)


def interrupt():
    initialize()
    for i in range(10):
        # Wait for some time
        rospy.sleep(10)
        # Send interruption (Pause)
        interrupt_goal.new_interrupt_state = 1
        interrupt_client.wait_for_server()
        interrupt_client.send_goal(interrupt_goal)
        interrupt_client.wait_for_result()
        interrupt_result = interrupt_client.get_result()
        # Wait for some time
        rospy.sleep(10)
        # Send interruption (Continue)
        interrupt_goal.new_interrupt_state = 0
        interrupt_client.wait_for_server()
        interrupt_client.send_goal(interrupt_goal)
        interrupt_client.wait_for_result()
        interrupt_result = interrupt_client.get_result()