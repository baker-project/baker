#!/usr/bin/env python

import rospy
import actionlib
import application_container
import behavior_container

class Interruptor():

    def __init__(self):
        print "Interruption module: Initializing..."
        # Create interruption client
        self.interrupt_goal = application_container.InterruptActionGoal()
        self.interrupt_goal.new_interrupt_state = 0
        print "Waiting for Action InterruptAction to become available..."
        self.interrupt_client = actionlib.SimpleActionClient("/interrupt_test_app", application_container.InterruptActionAction)
        print "Interruption module: Initialization completed"


    def interrupt(self):
        # rospy.sleep(10)
        for i in range(1):
            # Wait for some time
            rospy.sleep(1)
            # Send interruption (Pause)
            print "Interruption module: Pausing application..."
            self.interrupt_goal.new_interrupt_state = 1
            self.interrupt_client.wait_for_server()
            print "sending"
            self.interrupt_client.send_goal(self.interrupt_goal)
            print "waiting"
            self.interrupt_client.wait_for_result()
            self.interrupt_result = self.interrupt_client.get_result()
            # Wait for some time
            rospy.sleep(7)
            # Send interruption (Continue)
            print "Interruption module: Continuing application..."
            self.interrupt_goal.new_interrupt_state = 0
            self.interrupt_client.wait_for_server()
            self.interrupt_client.send_goal(self.interrupt_goal)
            self.interrupt_client.wait_for_result()
            self.interrupt_result = self.interrupt_client.get_result()

if __name__ == '__main__':
	try:
		rospy.init_node('application_test_interruptor_node')
		interruptor = Interruptor()
		interruptor.interrupt()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"




        #todo: shutdown for actionlib, pause != cancel