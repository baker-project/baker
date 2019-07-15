#! /usr/bin/env python

import rospy
import actionlib
from math import pi

from ipa_manipulation_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Transform

def executeAction(actionName, goal=ExecuteTrajectoryGoal()):
    client = actionlib.SimpleActionClient(actionName, ExecuteTrajectoryAction)
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def moveToPosition(joint_values):
    goal = ExecuteTrajectoryGoal()
    target_point = JointTrajectoryPoint()
    target_point.positions = joint_values
    goal.trajectory.joint_trajectory.points.append(target_point)
    return executeAction('baker_arm_module_interface/move_to_position', goal)

def moveToRestPosition():
    return executeAction('baker_arm_module_interface/rest_position')

def moveToTransportPosition():
    return executeAction('baker_arm_module_interface/transport_position')

def emptyTrashcan():
    return executeAction('baker_arm_module_interface/empty_trashcan')

def catchTrashcan():
    return executeAction('baker_arm_module_interface/catch_trashcan')

def leaveTrashcan():
    return executeAction('baker_arm_module_interface/leave_trashcan')

if __name__ == '__main__':

    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('baker_arm_client')
        print('Instructions')
        print('q: quit')
        print('r: go to rest position')
        print('t: go to transport position')
        print('m: move to given position')
        print('c: catch the trashcan')
        print('l: leave the trashcan')
        print('e: empty the trashcan')

        while True:
            print('Please select an option (q to quit)')
            choice = raw_input()
            if choice == 'q':
                break
            if choice == 'r':
                moveToRestPosition()
            if choice == 't':
                moveToTransportPosition()
            if choice == 'm':
                print('Please write the target positiont (joint values)')
                joint_values_str = raw_input()
                joint_values = [float(value) for value in joint_values_str[1:-1].split(',')]
                moveToPosition(joint_values)
            if choice == 'e':
                emptyTrashcan()
            if choice == 'l':
                leaveTrashcan()
            if choice == 'c':
                catchTrashcan()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
