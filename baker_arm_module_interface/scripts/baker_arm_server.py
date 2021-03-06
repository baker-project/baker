#!/usr/bin/env python

import rospy

from simple_script_server import script
from motion_planning import planTrajectoryInCartSpace,planTrajectoryInJointSpace, cartesianPlan, cartesianExecution
from motion_planning import ikService
from motion_planning import make_pose, make_pose_from_rpy, create_transformation_frame
from tf.transformations import euler_from_quaternion

from actionlib import SimpleActionServer, SimpleActionClient
from ipa_manipulation_msgs.msg import MoveToAction, MoveToGoal, MoveToResult
from ipa_manipulation_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal, ExecuteTrajectoryResult
from ipa_manipulation_msgs.msg import PlanToAction

from canopen_chain_node.srv import SetObject, SetObjectRequest, SetObjectResponse
from std_srvs.srv import Trigger, TriggerResponse
from ipa_manipulation_msgs.srv import AddCollisionObject, AddCollisionObjectResponse, AddCollisionObjectRequest, RemoveCollisionObject, RemoveCollisionObjectRequest, RemoveCollisionObjectResponse
from ipa_manipulation_msgs.msg import CollisionBox

from geometry_msgs.msg import PoseStamped, Point, Pose, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

import random
from math import pi, cos, sin
from threading import Thread, Lock
import sys

from enum import Enum

class ArmStatus(Enum):
    NO_TRASHCAN = 0
    EMPTY_TRASHCAN = 1
    FULL_TRASHCAN = 2

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
    IS_GRIPPER_AVAILABLE = False

    def __init__(self, name, status=ArmStatus.NO_TRASHCAN):
        self.verbose_ = True
        self.confirm_ = False
        self.name_ = name
        self.status_ = status
        self.joint_values_ = [0.0]*self.DOF
        self.mutex_ = Lock()
        (self.trashcan_pose_, self.trolley_pose_) = (None, None)

        self.displayParameters()
        self.initClients()
        self.initServices()
        self.initServers()
        self.initSubscribers()
        self.initTopics()

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
        # Warning: wait_for_server and wait_for_service are not the same function.
        # * rospy.duration expected for wait_for_server
        # * float expected for wait_for_service
        self.plan_client_ = SimpleActionClient('/arm_planning_node/PlanTo', PlanToAction)
        self.plan_client_.wait_for_server(timeout=rospy.Duration(5.0))

        self.execution_client_ = SimpleActionClient('/traj_exec', ExecuteTrajectoryAction)
        self.execution_client_.wait_for_server(timeout=rospy.Duration(5.0))

        self.small_gripper_finger_client_ = rospy.ServiceProxy("/gripper/driver/set_object", SetObject)
        # rmb-ma. keep commented
        # self.small_gripper_finger_client_.wait_for_service(timeout=5.0)

        self.add_collision_object_client_ = rospy.ServiceProxy("/ipa_planning_scene_creator/add_collision_objects", AddCollisionObject)
        self.add_collision_object_client_.wait_for_service(timeout=5.0)

        self.attach_object_client_ = rospy.ServiceProxy("ipa_planning_scene_creator/attach_object", AddCollisionObject)
        self.attach_object_client_.wait_for_service(timeout=5.0)

        self.detach_object_client_ = rospy.ServiceProxy("ipa_planning_scene_creator/detach_object", RemoveCollisionObject)
        self.detach_object_client_.wait_for_service(timeout=5.0)

    @log
    def initServers(self):
        self.to_joints_position_server_ =  SimpleActionServer(self.name_ + '/set_joints_values', ExecuteTrajectoryAction, self.moveToJointsPositionCallback, False)
        self.to_joints_position_server_.start()

        self.to_transport_position_server_ = SimpleActionServer(self.name_ + '/transport_position', MoveToAction, self.moveToTransportPositionCallback, False)
        self.to_transport_position_server_.start()

        self.to_rest_position_server_ = SimpleActionServer(self.name_ + '/rest_position', MoveToAction, self.moveToRestPositionCallback, False)
        self.to_rest_position_server_.start()

        self.catch_trashcan_server_ = SimpleActionServer(self.name_ + '/catch_trashcan', MoveToAction, self.catchTrashcanCallback, False)
        self.catch_trashcan_server_.start()

        self.leave_trashcan_server_ = SimpleActionServer(self.name_ + '/leave_trashcan', MoveToAction, self.leaveTrashcanCallback, False)
        self.leave_trashcan_server_.start()

        self.empty_trashcan_server_ = SimpleActionServer(self.name_ + '/empty_trashcan', MoveToAction, self.emptyTrashcanCallback, False)
        self.empty_trashcan_server_.start()

    @log
    def initServices(self):
        pass

    @log
    def initTopics(self):
        Thread(target=self.statusTalker).start()

    @log
    def statusTalker(self):
        publisher = rospy.Publisher(self.name_ + '/status', Int32, queue_size=10)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            publisher.publish(self.status_.value)
            rate.sleep()

    @log
    def initSubscribers(self):
        rospy.Subscriber("/arm/joint_states", JointState, self.jointStateCallback)

    def jointStateCallback(self, msg):
        '''
            getting current position of the joint
            @param msg containts joint position, velocity, effort, names
        '''
        for i in range(0,len(msg.position)):
            self.joint_values_[i] = msg.position[i]

    def gripperHandler(self, finger_value=0.01, finger_open=False, finger_close=True):
        '''
            Gripper handler function use to control gripper open/close command,
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

            response = self.small_gripper_finger_client_.call(request)
            if response.success:
                rospy.loginfo(response.message)
            else:
                rospy.logerr(response.message)
                rospy.logerr("SetObject service call Failed!!")

        # default blocking is true, wait until if finishes it tasks
        self.sss.move('gripper',[[finger_value]])
        return response.success

    @log
    def handleCheckAccessibility(self, request):
        response = TriggerResponse()

        response.success = True
        return response

    @log
    def openGripper(self):
        if not self.IS_GRIPPER_AVAILABLE:
            return
        self.gripperHandler(finger_value=0.01, finger_open=True, finger_close=False)

    @log
    def closeGripper(self):
        if not self.IS_GRIPPER_AVAILABLE:
            return
        self.gripperHandler(finger_value=-0.01, finger_open=False, finger_close=True)

    @log
    def executeTrajectory(self, trajectory, server):
        goal = ExecuteTrajectoryGoal()
        goal.trajectory = trajectory
        self.execution_client_.send_goal(goal)

        while self.execution_client_.get_state() < 3:
            rospy.sleep(0.1)
            if not server.is_preempt_requested() and not rospy.is_shutdown():
                continue
            self.execution_client_.cancel_goal()

            self.execution_client_.wait_for_result()
            return True

        result = self.execution_client_.get_result()
        state = self.execution_client_.get_state()

        if result.success and state == 3:
            rospy.logwarn("Execution finish with given time")
            return True
        else:
            rospy.logerr("Execution aborted!!")
            return False

    @log
    def isPoseAccessible(self, target_pose):
        (_, is_planned) = planTrajectoryInCartSpace(client=self.plan_client_, object_pose=target_pose, bdmp_goal=None, bdmp_action='')
        return is_planned

    @log
    def planAndExecuteTrajectoryInJointSpaces(self, target_joints, server):
        (trajectory, is_planned) = planTrajectoryInJointSpace(client=self.plan_client_, object_pose=target_joints,  bdmp_goal=None, bdmp_action='')
        if not is_planned:
            rospy.logerr('Trajectory in joint spaces execution failed')
            raise Exception('Trajectory in joint spaces execution failed')
        self.confirm()
        is_execution_successful = self.executeTrajectory(trajectory=trajectory, server=server)
        if not is_execution_successful:
            rospy.logerr('Can not find valid trajectory at joint space')
            raise Exception('Trajectory Execution failed')
        return not server.is_preempt_requested()

    @log
    def planAndExecuteTrajectoryInCartesianSpace(self, target_pose, server):
        (trajectory, is_planned) = planTrajectoryInCartSpace(client=self.plan_client_, object_pose=target_pose, bdmp_goal=None, bdmp_action='')
        if not is_planned:
            rospy.logerr('Cannot find valid trajectory at cartesianSpace')
            raise Exception('Cannot find valid trajectory at cartesian space')

        self.confirm()
        is_execution_successful = self.executeTrajectory(trajectory=trajectory, server=server)
        if not is_execution_successful:
            rospy.logerr("Trajectory Execution Failed")
            raise Exception('Trajectory Execution Failed')

        return not server.is_preempt_requested()

    @staticmethod
    def poseToLists(pose):
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return (position, orientation)


    @log
    def catchTrashcanCallback(self, goal):
        rospy.sleep(3)
        result = MoveToResult()
        result.arrived = False

        target_pose = goal.target_pos
        target_pose.pose.position.z *= 1.1

        #The arm canot carry a new trashcan as it already carries one
        if self.status_ != ArmStatus.NO_TRASHCAN:
            self.catch_trashcan_server_.set_aborted(result)

        try:
            self.planAndExecuteTrajectoryInCartesianSpace(target_pose, self.catch_trashcan_server_)
        except Exception as e:
            print(e)
            self.catch_trashcan_server_.set_aborted(result)
            return

        if self.catch_trashcan_server_.is_preempt_requested():
            result.arrived = True
            self.catch_trashcan_server_.set_preempted(result)
            return

        self.closeGripper()
        #self.fixTrashCanToRobot()
        self.status_ = ArmStatus.FULL_TRASHCAN

        # todo rmb-ma setup the grapping move with a real trashcan
        # Help:
        # try:
        #     target_pose = make_pose(position=[0.00,-0.00,-0.07], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        #     self.planAndExecuteTrajectoryInCartesianSpace(target_pose, self.catch_trashcan_server_)
        # except:
        #     self.catch_trashcan_server_.set_aborted(result)
        #     return
        #
        # if self.catch_trashcan_server_.is_preempt_requested():
        #     result.arrived = True
        #     self.catch_trashcan_server_.set_preempted(result)
        #     return
        #
        # try:
        #     target_pose = make_pose(position=[0.020,-0.00,0.00], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        #     self.planAndExecuteTrajectoryInCartesianSpace(target_pose, self.catch_trashcan_server_)
        # except:
        #     self.catch_trashcan_server_.set_aborted(result)
        #     return
        #
        # if self.catch_trashcan_server_.is_preempt_requested():
        #     result.arrived = True
        #     self.catch_trashcan_server_.set_preempted(result)
        #     return
        #
        # try:
        #     target_pose = make_pose(position=[0.000,-0.00,0.030], orientation=[0.0,0.0,0.0,1.0], frame_id='gripper')
        #     self.planAndExecuteTrajectoryInCartesianSpace(target_pose, self.catch_trashcan_server_)
        # except:
        #     self.catch_trashcan_server_.set_aborted(result)
        #     return
        #
        result.arrived = True
        self.catch_trashcan_server_.set_succeeded(result)

    @log
    def emptyTrashcanCallback(self, goal):
        result = MoveToResult()
        #Trashcan emptying while the arm doesnt carry any trashcan
        if self.status_ != ArmStatus.FULL_TRASHCAN:
            result.arrived = False
            self.empty_trashcan_server_.set_aborted(result)
            return

        try:
            self.planAndExecuteTrajectoryInCartesianSpace(goal.target_pos, self.empty_trashcan_server_)
        except:
            result.arrived = False
            self.empty_trashcan_server_.set_aborted(result)
            return

        if self.empty_trashcan_server_.is_preempt_requested():
            self.empty_trashcan_server_.set_preempted()
            return

        try:
            target_joints = self.joint_values_[0:-1] + [3.]
            self.planAndExecuteTrajectoryInJointSpaces(target_joints, self.empty_trashcan_server_)
        except:
            result.arrived = False
            self.empty_trashcan_server_.set_aborted(result)
            return

        if self.empty_trashcan_server_.is_preempt_requested():
            self.empty_trashcan_server_.set_preempted()
            return

        rospy.sleep(5)
        self.status_ = ArmStatus.FULL_TRASHCAN
        target_joints = self.joint_values_[0:-1] + [0]
        self.planAndExecuteTrajectoryInJointSpaces(target_joints, self.empty_trashcan_server_)

        result.arrived = True
        self.empty_trashcan_server_.set_succeeded(result)

    @log
    def moveToRestPositionCallback(self, goal):
        result = MoveToResult()
        # Rest position is unavailable if the arm carries a trashcan
        if self.status_ != ArmStatus.NO_TRASHCAN:
            result.arrived = False
            self.to_rest_position_server_.set_aborted(result)
            return

        try:
            self.planAndExecuteTrajectoryInJointSpaces(self.DEFAULT_POSITION, self.to_rest_position_server_)
        except Exception as e:
            rospy.logerr(e);
            result.arrived = False
            self.to_rest_position_server_.set_aborted(result)
            return

        if self.to_rest_position_server_.is_preempt_requested():
            self.to_rest_position_server_.set_preempted()
            return

        self.to_rest_position_server_.set_succeeded()

    @log
    def moveToTransportPositionCallback(self, goal):
        result = MoveToResult()
        # Transport position is unavailable if the arm doesnt carry a trashcan
        if self.status_ == ArmStatus.NO_TRASHCAN:
            result.arrived = False
            self.to_transport_position_server_.set_aborted(result)
            return

        try:
            result.arrived = self.planAndExecuteTrajectoryInJointSpaces(self.TRANSPORT_POSITION, self.to_transport_position_server_)
        except:
            result.arrived = False
            self.to_transport_position_server_.set_aborted(result)
            return

        if self.to_transport_position_server_.is_preempt_requested():
            self.to_transport_position_server_.set_preempted()
            return

        self.to_transport_position_server_.set_succeeded(result)

    @log
    def leaveTrashcanCallback(self, goal):
        result = MoveToResult()
        # The arm cannot leave a trashcan if it doesnt carry one
        if self.status_ != ArmStatus.EMPTY_TRASHCAN:
            result.arrived = False
            self.leave_trashcan_server_.set_aborted(result)
            return

        self.planAndExecuteTrajectoryInCartesianSpace(goal.target_pos, self.leave_trashcan_server_)

        if self.leave_trashcan_server_.is_preempt_requested():
            self.leave_trashcan_server_.set_preempted()
            return

        self.openGripper()
        self.status_ = ArmStatus.NO_TRASHCAN

        if self.leave_trashcan_server_.is_preempt_requested():
            self.leave_trashcan_server_.set_preempted()
            return

        target_pose = make_pose(position=[-0.07, -0., -0.04], orientation=[0., 0., 0., 1.], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose, self.leave_trashcan_server_)

        if self.leave_trashcan_server_.is_preempt_requested():
            self.leave_trashcan_server_.set_preempted()
            return

        target_pose = make_pose(position=[-0.0, -0., 0.05], orientation=[0., 0., 0., 1.], frame_id='gripper')
        self.planAndExecuteTrajectoryInCartesianSpace(target_pose, self.leave_trashcan_server_)

        if self.leave_trashcan_server_.is_preempt_requested():
            self.leave_trashcan_server_.set_preempted()
            return

        result.arrived = True
        self.leave_trashcan_server_.set_succeeded(result)

    @log
    def moveToJointsPositionCallback(self, goal):
        target_joints = goal.trajectory.joint_trajectory.points[0].positions
        self.planAndExecuteTrajectoryInJointSpaces(target_joints, self.to_joints_position_server_)

        if self.to_joints_position_server_.is_preempt_requested():
            self.to_joints_position_server_.set_preempted()
            return

        self.to_joints_position_server_.set_succeeded()

if __name__ == "__main__":

    try:
        rospy.init_node('baker_arm_module_interface', anonymous=True)
        BakerArmServer('baker_arm_module_interface')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
