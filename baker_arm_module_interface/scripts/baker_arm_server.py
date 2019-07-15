#!/usr/bin/env python

import rospy
import sys
from actionlib import SimpleActionServer, SimpleActionClient
from threading import Thread, Lock

from sensor_msgs.msg import JointState

from motion_planning import planTrajectoryInCartSpace,planTrajectoryInJointSpace, executionActionServer, cartesianPlan, cartesianExecution
from motion_planning import loadEnvironment, unloadEnvironment, addCollisionObject, removeCollisionObject, attachObject, detachObject
from motion_planning import ikService
from motion_planning import make_bounding_box, make_pose, make_pose_from_rpy, create_transformation_frame, convertRPYToQuat

from ipa_manipulation_msgs.msg import CollisionBox, PlanToAction, MoveToAction, ExecuteTrajectoryAction
from ipa_manipulation_msgs.srv import AddCollisionObject, RemoveCollisionObject

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped, Point, Pose

import random
from math import pi

from simple_script_server import script

def log(funct):
    def funct_wrapper(self, *args, **kwargs):
        if self.verbose_:
            print("Executing {}".format(funct.__name__))
        return funct(self, *args, **kwargs)
    return funct_wrapper


class BakerArmServer(script):

    DOF = 5
    DEFAULT_POSITION = [0.6, 1., -2., 1., 0.]
    TRANSPORT_POSITION = [0.9, 1., -2., 1., 0.]

    def __init__(self, name):

        self.verbose_ = True
        self.confirm_ = False
        self.name_ = name
        self.displayParameters()
        self.initServices()
        self.initServers()
        self.initClients()
        self.initSubscribers()

        self.joint_values_ = [0.0]*self.DOF

        self.mutex_ = Lock()

    def confirm(self):
        if not self.confirm_:
            return
        raw_input("Please press enter to confirm")

    def displayParameters(self):
        if not self.verbose_:
            return
        print("========== baker_arm_server Parameters ==========")
        print("todo")

    @log
    def Initialize(self):
        self.sss.init('gripper')
        self.sss.init('arm')

    @log
    def initClients(self):
        self.plan_client_ = SimpleActionClient('/arm_planning_node/PlanTo', PlanToAction)
        self.plan_client_.wait_for_server(rospy.Duration(5.0))
        self.execution_client_ = SimpleActionClient('/traj_exec', ExecuteTrajectoryAction)
        self.execution_client_.wait_for_server(timeout=rospy.Duration(5.0))

    @log
    def initServers(self):
        self.to_joints_position_server_ =  SimpleActionServer(self.name_ + '/move_to_position', ExecuteTrajectoryAction, self.moveToJointsPositionCallback, False)
        self.to_joints_position_server_.start()

        self.to_transport_position_server_ = SimpleActionServer(self.name_ + '/transport_position', ExecuteTrajectoryAction, self.moveToTransportPositionCallback, False)
        self.to_transport_position_server_.start()

        self.to_rest_position_server_ = SimpleActionServer(self.name_ + '/rest_position', ExecuteTrajectoryAction, self.moveToRestPositionCallback, False)
        self.to_rest_position_server_.start()

        self.catch_trashcan_server_ = SimpleActionServer(self.name_ + '/catch_trashcan', ExecuteTrajectoryAction, self.catchTrashcanCallback, False)
        self.catch_trashcan_server_.start()

        self.leave_trashcan_server_ = SimpleActionServer(self.name_ + '/leave_trashcan', ExecuteTrajectoryAction, self.leaveTrashcanCallback, False)
        self.leave_trashcan_server_.start()

        self.empty_trashcan_server_ = SimpleActionServer(self.name_ + '/empty_trashcan', ExecuteTrajectoryAction, self.emptyTrashcanCallback, False)
        self.empty_trashcan_server_.start()

    @log
    def initServices(self):
        # todo rmb-ma delete if still empty
        pass

    @log
    def initSubscribers(self):
        rospy.Subscriber("/arm/joint_states", JointState, self.jointStateCallback)

    def jointStateCallback(self,msg):
        '''
            getting current position of the joint
            @param msg containts joint position, velocity, effort, names
        '''
        for i in range(0,len(msg.position)):
            self.joint_values_[i] = msg.position[i]

    def gripperHandler(self, client, finger_value=0.01, finger_open=False, finger_close=True):
        '''
            Gripper handler function use to control gripper open/colse command,
            move parallel part of gripper with give values (in cm)
            @param finger_value is the big parallel part move realtive to zero position
            @finger_open open the small finger
            @finger_close close the small finger
        '''
        if finger_open or finger_close:
            # set_object request, both the finger move together (at the same time)
            request = SetObjectRequest()
            response = SetObjectResponse()
            request.node = 'gripper_finger1_joint'
            request.object = '22A0'
            if finger_open:
                request.value = '5'
            elif finger_close:
                request.value = '10'
            request.cached = False

            response = client.call(request)
            if response.success:
                rospy.loginfo(response.message)
            else:
                rospy.logerr(response.message)
                rospy.logerr("SetObject service call Failed!!")

        # default blocking is true, wait until if finishes it tasks
        self.sss.move('gripper',[[finger_value]])
        return response.success

    @log
    def planAndExecuteTrajectoryInJointSpaces(self, target_joints):
        (trajectory, is_planned) = planTrajectoryInJointSpace(client=self.plan_client_, object_pose=target_joints,  bdmp_goal=None, bdmp_action='')
        if not is_planned:
            rospy.logerr('Trajectory in joint spaces execution failed')
            raise Exception('Trajectory in joint spaces execution failed')
        self.confirm()
        is_execution_successful = executionActionServer(client=self.execution_client_, trajectory=trajectory)
        if not is_execution_successful:
            rospy.logerr('Can not find valid trajectory at joint space')
            raise Exception('Trajectory Execution failed')

    @log
    def planAndExecuteTrajectoryInCartesianSpace(self, target_pose):
        (trajectory, is_planned) = planTrajectoryInCartSpace(client=self.plan_client_, object_pose=target_pose, bdmp_goal=None, bdmp_action='')
        if not is_planned:
            rospy.logerr('Cannot find valid trajectory at cartesianSpace')
            raise Exception('Cannot find valid trajectory at cartesian space')
        self.confirm()
        is_execution_successful = executionActionServer(client=self.execution_client_, trajectory=trajectory)
        if not is_execution_successful:
            rospy.logerr("Trajectory Execution Failed")
            raise Exception('Trajectory Execution Failed')

    @log
    def catchTrashcanCallback(self, goal):
        rospy.loginfo("Plan trajectory and move to pick dustbin ...")

        # rpy means roll - pitch - yaw
        # 1.
        target_pose = make_pose_from_rpy(position=[0.75,-0.60, 0.505], rotation=[0.0, 0.0, 3.92], frame_id='world')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        # 2.
        target_pose = make_pose(position=[0.00,-0.00,-0.07], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        # 3.
        target_pose = make_pose(position=[0.020,-0.00,0.00], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        # 4.
        target_pose = make_pose(position=[0.000,-0.00,0.030], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        self.catch_trashcan_server_.set_succeeded()

    @log
    def emptyTrashcanCallback(self, goal):
        target_pose = make_pose_from_rpy(position=[0.800, 0.600, 0.80], rotation=[-0.000, 0.000, 0.401, 0.916], frame_id='world')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        target_joints = self.joint_values_[0:-1] + [3.]
        self.planAndExecuteTrajectoryInJointSpaces(target_joints)

        rospy.sleep(5)
        target_joints = self.joint_values_[0:-1] + [0]
        self.planAndExecuteTrajectoryInJointSpaces(target_joints)
        self.empty_trashcan_server_.set_succeeded()

    @log
    def moveToRestPositionCallback(self, goal):
        self.planAndExecuteTrajectoryInJointSpaces(self.DEFAULT_POSITION)
        self.to_rest_position_server_.set_succeeded()

    @log
    def moveToTransportPositionCallback(self, goal):
        self.planAndExecuteTrajectoryInJointSpaces(self.TRANSPORT_POSITION)
        self.to_transport_position_server_.set_succeeded()

    @log
    def leaveTrashcanCallback(self, goal):

        target_pose = make_pose_from_rpy(position=[0.75,-0.60, 0.505], rotation=[0.0, 0.0, 3.92], frame_id='world')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        target_pose = make_pose(position=[0.000,-0.00,0.030], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        target_pose = make_pose(position=[0.020,-0.00,0.00], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

        target_pose = make_pose(position=[0.00,-0.00,-0.07], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose)

    @log
    def moveToJointsPositionCallback(self, goal):
        target_joints = goal.trajectory.joint_trajectory.points[0].positions
        print(target_joints)
        self.planAndExecuteTrajectoryInJointSpaces(target_joints)
        self.to_joints_position_server_.set_succeeded()

if __name__ == "__main__":

    try:
        rospy.init_node('baker_arm_module_interface', anonymous=True)
        BakerArmServer('baker_arm_module_interface')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
