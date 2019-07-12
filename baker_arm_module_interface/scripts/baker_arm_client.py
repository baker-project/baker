#! /usr/bin/env python

import rospy
import actionlib
from math import pi

from ipa_manipulation_msgs.msg import ExecuteTrajectoryAction

def move_to_position(joint_values):
    print("waiting for the server")
    client = actionlib.SimpleActionClient('baker_arm_module_interface/move_to_position', ExecuteTrajectoryAction)
    client.wait_for_server()

    goal = ExecuteTrajectoryAction()
    goal.RobotTrajectory.multi_dof_joint_trajectory.transforms = [0.8, pi/2., -2, 0., 0.]
    print("goal")
    client.send_goal(goal)
    print("waiting for the result")
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('baker_arm_client')
        result = move_to_position([])
        print("ok")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
