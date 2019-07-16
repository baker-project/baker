#! /usr/bin/env python

import rospy
import actionlib
from math import pi

from ipa_manipulation_msgs.msg import MoveToAction, MoveToGoal, ExecuteTrajectoryGoal, ExecuteTrajectoryAction

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Transform
from tf.transformations import quaternion_from_euler

def executeAction(actionName, goal=MoveToGoal(), action_type=MoveToAction):
    client = actionlib.SimpleActionClient(actionName, action_type)
    client.wait_for_server()
    client.send_goal(goal)
    ## Pause testing purpose
    #rospy.sleep(10)
    #if client.get_state() != 3:
    #    client.cancel_goal()
    client.wait_for_result()
    print("RESULT {}".format(client.get_result()))
    return client.get_result()

def setJointsValues(joint_values):
    goal = MoveToGoal()
    target_point = JointTrajectoryPoint()
    target_point.positions = joint_values
    goal.trajectory.joint_trajectory.points.append(target_point)
    return executeAction('baker_arm_module_interface/move_to_position', goal, action_type=ExecuteTrajectoryAction)

def moveToRestPosition():
    return executeAction('baker_arm_module_interface/rest_position')

def moveToTransportPosition():
    return executeAction('baker_arm_module_interface/transport_position')

def createGoal(position, rotation, frame_id):
    goal = MoveToGoal()
    orientation = quaternion_from_euler(*rotation)

    goal.target_pos.pose.position.x = position[0]
    goal.target_pos.pose.position.y = position[1]
    goal.target_pos.pose.position.z = position[2]

    goal.target_pos.pose.orientation.x = orientation[0]
    goal.target_pos.pose.orientation.y = orientation[1]
    goal.target_pos.pose.orientation.z = orientation[2]
    goal.target_pos.pose.orientation.w = orientation[3]

    goal.target_pos.header.frame_id = frame_id
    return goal

def emptyTrashcan():
    goal = createGoal(position=[0.800, 0.600, 0.80], rotation=[-0., 0., 0.401], frame_id='world')
    return executeAction('baker_arm_module_interface/empty_trashcan', goal=goal, action_type=MoveToAction)

def catchTrashcan():
    goal = createGoal(position=[0.75, -0.60, 0.505], rotation=[0., 0., 3.92], frame_id='world')
    return executeAction('baker_arm_module_interface/catch_trashcan', goal=goal, action_type=MoveToAction)

def leaveTrashcan():
    goal = createGoal(position=[0.75,-0.60, 0.505], rotation=[0.0, 0.0, 3.92], frame_id='world')
    return executeAction('baker_arm_module_interface/leave_trashcan', goal=goal, action_type=MoveToAction)

if __name__ == '__main__':
    try:
        rospy.init_node('baker_arm_client')
        print('Instructions')
        print('q: quit')
        print('r: go to rest position')
        print('t: go to transport position')
        print('j: set joins values')
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
            if choice == 'j':
                print('Please write the target positiont (joint values)')
                joint_values_str = raw_input()
                joint_values = [float(value) for value in joint_values_str[1:-1].split(',')]
                setJointsValues(joint_values)
            if choice == 'e':
                emptyTrashcan()
            if choice == 'l':
                leaveTrashcan()
            if choice == 'c':
                catchTrashcan()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
