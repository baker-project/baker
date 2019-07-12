#!/usr/bin/env python

import rospy
import actionlib

#msg import
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.msg import RobotTrajectory

from actionlib.msg import TestAction, TestGoal
from actionlib import GoalStatus
from canopen_chain_node.srv import SetObject, SetObjectRequest, SetObjectResponse

from numpy import float64

import tf
from tf.transformations import quaternion_from_euler
import tf2_ros
import yaml

from numpy import linalg as linalg
from ipa_manipulation_msgs.msg import *
from ipa_manipulation_msgs.srv import *
from kinematic_calculation.msg import *
from kinematic_calculation.srv import *
import ros

# uplaod parameter from yaml file
import roslib
roslib.load_manifest("rosparam")
import rosparam
import rospkg
import argparse
from numpy import linalg as LA

#from test_common import make_bounding_box
# from test_common import publish_transform_msg

def make_bounding_box(x, y, z):
    # setting up object size
    bounding_box_lwh = Vector3()
    bounding_box_lwh.x = 0.08 if x is None else x
    bounding_box_lwh.y = 0.12 if y is None else y
    bounding_box_lwh.z = 0.20 if z is None else z
    return bounding_box_lwh

def make_pose_from_rpy(position, rotation, frame_id):
    orientation = convertRPYToQuat(roll=rotation[0], pitch=rotation[1], yaw=rotation[2])
    return make_pose(position, orientation, frame_id)

def make_pose(position, orienatation, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]

    pose.pose.orientation.x = orienatation[0]
    pose.pose.orientation.y = orienatation[1]
    pose.pose.orientation.z = orienatation[2]
    pose.pose.orientation.w = orienatation[3]
    return pose

def convertRPYToQuat(roll=0.0, pitch=0.0, yaw=0.0):
    return quaternion_from_euler(roll, pitch, yaw)

def create_transform_msg(parent_frame_name, frame_name, pose, orientation):
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = parent_frame_name

    static_transform_stamped.child_frame_id = frame_name

    static_transform_stamped.transform.translation.x = pose[0]
    static_transform_stamped.transform.translation.y = pose[1]
    static_transform_stamped.transform.translation.z = pose[2]

    norm_orient = linalg.norm(orientation)
    static_transform_stamped.transform.rotation.x = float(orientation[0] / norm_orient)
    static_transform_stamped.transform.rotation.y = float(orientation[1] / norm_orient)
    static_transform_stamped.transform.rotation.z = float(orientation[2] / norm_orient)
    static_transform_stamped.transform.rotation.w = float(orientation[3] / norm_orient)

    return static_transform_stamped

def create_transformation_frame(child_frame_id='', object_pose = PoseStamped()):
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.child_frame_id = child_frame_id
    static_transformStamped.header.stamp = object_pose.header.stamp
    static_transformStamped.header.frame_id = object_pose.header.frame_id

    # create frame
    static_transformStamped.transform.translation.x = object_pose.pose.position.x
    static_transformStamped.transform.translation.y = object_pose.pose.position.y
    static_transformStamped.transform.translation.z = object_pose.pose.position.z
    orientation = [object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z, object_pose.pose.orientation.w]
    norm_orient = LA.norm(orientation)

    # print norm_orient
    static_transformStamped.transform.rotation.x = float(orientation[0] / norm_orient)
    static_transformStamped.transform.rotation.y = float(orientation[1] / norm_orient)
    static_transformStamped.transform.rotation.z = float(orientation[2] / norm_orient)
    static_transformStamped.transform.rotation.w = float(orientation[3] / norm_orient)


    # broadcast all frame together
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(static_transformStamped)

def publish_transform_msg(param_name=''):
    frames_list = rospy.get_param(param_name)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    brd_list = []

    for name, data in frames_list.items():
        msg = create_transform_msg(parent_frame_name=data['goal_frame']['frame_id'],
                                   frame_name=name + '_goal_frame',
                                   pose=data['goal_frame']['position'],
                                   orientation=data['goal_frame']['orientation'])
        brd_list.append(msg)

        if 'start_frame' in data:
            msg = create_transform_msg(parent_frame_name=data['start_frame']['frame_id'],
                                       frame_name=name + '_start_frame',
                                       pose=data['start_frame']['position'],
                                       orientation=data['start_frame']['orientation'])
        brd_list.append(msg)

    broadcaster.sendTransform(brd_list)


def ikService(service_name='', base_link='', joint_values=[], eef_pose=PoseStamped()):
    """
        test computation of FK computation
        @:param service name: name used to call service
        @:param base_link: link name used for header, all transformation matrix generate accord this header frame
        @:param joint_values: requested joint value to compute fk
    """
    # create service client for fk computation
    rospy.wait_for_service(service_name)
    client = rospy.ServiceProxy(service_name, ComputeIK)

    # make service request
    # fill header for request
    request = ComputeIKRequest()

    # fill joint state/values
    requested_joint_state = JointState()
    # no requested joint value than take current joint values
    if len(joint_values) is 0:
        pass

    else:
        # push requested joint values
        requested_joint_state.position = []
        for value in joint_values:
            requested_joint_state.position.append(value)
    request.current_joint_values = requested_joint_state

    print request.current_joint_values

    #eef pose
    request.eef_pose_stamped = eef_pose

    # send request for fk calculation
    response = client.call(request)

    # display results on the terminal
    if response.ik_success is True:
        rospy.loginfo(response.message)
        for index, value in enumerate(response.joint_names):
            print index,value

        for index, value in enumerate(response.joint_values):
            print index,value

        return response.joint_values, response.ik_success
    else:
        rospy.logerr(response.message)

    return JointState(), response.ik_success

        #rospy.loginfo(str(fk_pose))

    #rospy.loginfo("eef value: \n %s", str(response.eef_pose_stamped))


def make_add_collision_request(label='', id= '', size=Vector3(), object_pose=PoseStamped(), method='primitive'):
    # make collision object form of msgs
    co_box = CollisionBox()
    co_box.object_id = label + id
    co_box.id = id
    co_box.label = label
    co_box.pose = object_pose
    co_box.bounding_box_lwh = make_bounding_box(size.x, size.y, size.z)

    # make request for loading environment
    request = AddCollisionObjectRequest()
    request.loading_method = method #'primitive' # mesh
    request.collision_objects.append(co_box)

    return request


def make_remove_colliison_request(label='', id=''):
    # make request to remove environment collision object
    request = RemoveCollisionObjectRequest()
    request.object_id = label + id
    request.label = label
    request.loading_method = 'primitive' #mesh
    request.object_ids.append(label+id)
    return request

def loadEnvironment(client, label='', id= '', object_pose=PoseStamped()):

    # send request for loading environment, blocking untill response
    if client.call(make_add_collision_request(label=label, id=id, object_pose=object_pose)) is True:

        # get response of loading environments
        response = AddCollisionObjectResponse()

        if response.success is True:
            rospy.loginfo(response.msg)
        else:
            rospy.logerr(response.msg)


def unloadEnvironment(client, label='', id=''):

    # send request for adding collision environment, blocking untill response
    if client.call(make_remove_colliison_request(label=label, id=id)) is True:

        # get response of unloding environment
        response = RemoveCollisionObjectResponse()
        if response.success is True:
            rospy.loginfo(response.msg)
        else:
            rospy.logerr(response.msg)

def addCollisionObject(client, label='', id= '', size=Vector3(), object_pose=PoseStamped(),method='primitive'):

    # send request for loading environment
    if client.call(make_add_collision_request(label=label, id=id, size=size, object_pose=object_pose, method=method)) is True:

        # get response of loading environments
        response = AddCollisionObjectResponse()
        if response.success is True:
            rospy.loginfo(response.msg)
        else:
            rospy.logerr(response.msg)


def removeCollisionObject(client, label='', id=''):
    # send request for adding collision environment
    if client.call(make_remove_colliison_request(label=label, id=id)) is True:

        # get response of unloding environment
        response = RemoveCollisionObjectResponse()
        if response.success is True:
            rospy.loginfo(response.msg)
        else:
            rospy.logerr(response.msg)

def removeAllCollisionObject(client, label='', id=''):

    # send request for adding collision environment
    request = TriggerRequest()
    if client.call(request) is True:

        # get response of unloding environment
        response = TriggerResponse()
        if response.success is True:
            rospy.loginfo(response.message)
        else:
            rospy.logerr(response.message)


def attachObject(client, label='', id= '', object_pose=PoseStamped()):

    # send request for loading environment
    if client.call(make_add_collision_request(label=label, id=id, object_pose=object_pose)) is True:

        # get response of loading environments
        response = AddCollisionObjectResponse()
        if response.success is True:
            rospy.loginfo(response.msg)
        else:
            rospy.logerr(response.msg)


def detachObject(client, label='', id=''):

    # send request for adding collision environment
    if client.call(make_remove_colliison_request(label=label, id=id)) is True:

        # get response of unloding environment
        response = RemoveCollisionObjectResponse()
        if response.success is True:
            rospy.loginfo(response.msg)
        else:
            rospy.logerr(response.msg)


def planTrajectoryInCartSpace(client, bdmp_goal=None, bdmp_action='', object_pose=PoseStamped()):
    # plan trajectory action client
    #client = actionlib.SimpleActionClient('/arm_planning_node/PlanTo', ipa_manipulation_msgs.msg.PlanToAction)
    #client.wait_for_server(rospy.Duration(5.0))

    # make plan trajectory in cartesian space request
    goal = ipa_manipulation_msgs.msg.PlanToGoal()
    goal.expected_poseStamped = object_pose
    goal.action = bdmp_action
    # goal.bdmp_goal = bdmp_goal
    goal.bdmp_goal.request.group_name = ''

    # send goal
    client.send_goal(goal)

    time = client.wait_for_result(rospy.Duration.from_sec(50.))
    state = client.get_state()
    result = client.get_result()

    valid = result.got_plan

    if (time and result.got_plan and state is 3):
        rospy.loginfo('===== planTrajectoryInCartSpace: SUCCESSED =======')
        rospy.loginfo(result.message)
        rospy.loginfo(result.trajectory_planning_time)
        return result.planned_trajectory, valid

    return RobotTrajectory(), valid

def planTrajectoryInJointSpace(client, bdmp_goal=None, bdmp_action='', object_pose=list()):
    # plan trajectory action client
    #client = actionlib.SimpleActionClient('/arm_planning_node/PlanTo', ipa_manipulation_msgs.msg.PlanToAction)
    #client.wait_for_server(rospy.Duration(5.0))

    # make plan trajectory in cartesian space request
    goal = ipa_manipulation_msgs.msg.PlanToGoal()
    #goal.expected_joint_values = object_pose
    #goal.expected_joint_values = list()
    for val in object_pose:
        goal.expected_joint_values.append(val)
    goal.action = bdmp_action
    goal.bdmp_goal.request.group_name = ''
    # goal.bdmp_goal = bdmp_goal

    # send goal
    client.send_goal(goal)

    time = client.wait_for_result(rospy.Duration.from_sec(50.))
    state = client.get_state()
    result = client.get_result()

    valid = result.got_plan

    if (time and result.got_plan and state is 3):
        rospy.loginfo('===== planTrajectoryInCartSpace: SUCCESSED =======')
        rospy.loginfo(result.message)
        rospy.loginfo(result.trajectory_planning_time)
        return result.planned_trajectory, valid
    else:
        rospy.logerr("planTrajectoryInJointSpace Failed to find trajectory")
        return RobotTrajectory(), False

def executionTrajectory(follow_plan=False, object_pose=PoseStamped()):
    # execute trajectory action client
    client = actionlib.SimpleActionClient('/arm_planning_node/MoveTo', ipa_manipulation_msgs.msg.MoveToAction)
    client.wait_for_server(rospy.Duration(5.0))

    # make plan trajectory in cartesian space request
    goal = ipa_manipulation_msgs.msg.MoveToGoal()
    goal.followed_from_executed_trajectory = follow_plan
    goal.target_pos = object_pose

    # send goal
    client.send_goal(goal)

    time = client.wait_for_result(rospy.Duration.from_sec(50.))
    state = client.get_state()
    result = client.get_result()

    if (time and result.arrived and state is 3):
        rospy.loginfo('===== executionTrajectory: SUCCESSED =======')


def cartesianPlan(bdmp_goal=None, bdmp_action='', object_pose=PoseStamped()):
    # plan trajectory action client
    client = actionlib.SimpleActionClient('/arm_planning_node/PlanInCartesian', ipa_manipulation_msgs.msg.PlanToAction)
    client.wait_for_server(rospy.Duration(5.0))

    # make plan trajectory in cartesian space request
    goal = ipa_manipulation_msgs.msg.PlanToGoal()
    goal.expected_poseStamped = object_pose
    goal.action = bdmp_action
    # goal.bdmp_goal = bdmp_goal
    goal.bdmp_goal.request.group_name = ''

    # send goal
    client.send_goal(goal)

    time = client.wait_for_result(rospy.Duration.from_sec(50.))
    state = client.get_state()
    result = client.get_result()

    if (time and result.got_plan and state is 3):
        rospy.loginfo('===== cartesianPlan: SUCCESSED =======')
        rospy.loginfo(result.message)
        rospy.loginfo(result.trajectory_planning_time)
        return result.planned_trajectory, True
    else:
        rospy.logerr("planTrajectoryInJointSpace Failed to find trajectory")
        return RobotTrajectory(), False


def cartesianExecution(follow_plan=False, object_pose=PoseStamped()):
    # execute cartesian trajectory action client
    client = actionlib.SimpleActionClient('/arm_planning_node/MoveInCartesian', ipa_manipulation_msgs.msg.MoveToAction)
    client.wait_for_server(rospy.Duration(5.0))

    # make plan trajectory in cartesian space request
    goal = ipa_manipulation_msgs.msg.MoveToGoal()
    goal.followed_from_executed_trajectory = follow_plan
    goal.target_pos = object_pose

    # send goal
    client.send_goal(goal)

    time = client.wait_for_result(rospy.Duration.from_sec(50.))
    state = client.get_state()
    result = client.get_result()

    if (time and result.arrived and state is 3):
        rospy.loginfo('===== executionTrajectory: SUCCESSED =======')

def executionActionServer(client, trajectory=RobotTrajectory):

    print "##### Starting executionActionServer #######"

    #for i in trajecotry.joint_trajectory.joint_names:
    #    print i

    #print "!!!!!!!!!!!!!!!!!!!!"

    # initialize client and wait for start it
    #client = actionlib.SimpleActionClient('/traj_exec', ExecuteTrajectoryAction)
    #client.wait_for_server(timeout=rospy.Duration(5.0))
    #print "------------"
    # create goal for action server
    goal = ipa_manipulation_msgs.msg.ExecuteTrajectoryGoal()

    goal.trajectory = trajectory

    # send goal
    client.send_goal(goal)

    # wait for results
    time = client.wait_for_result(timeout=rospy.Duration(100.0))

    result = client.get_result()
    state = client.get_state()

    if (time and result.success and state == 3):
        rospy.logwarn("Execution finish with given time")
        return True
    else:
        rospy.logerr("Execution aborted!!")
        return False

    #print result.message
def mainPlanningScene(object_label = '', object_id = '0', object_pose=PoseStamped()):

    #############################################################
    # client initialization

    # waiting for service become available
    rospy.wait_for_service("/ipa_planning_scene_creator/load_environment")
    rospy.wait_for_service("/ipa_planning_scene_creator/unload_environment")
    rospy.wait_for_service("/ipa_planning_scene_creator/add_collision_objects")
    rospy.wait_for_service("/ipa_planning_scene_creator/remove_collision_objects")
    rospy.wait_for_service("/ipa_planning_scene_creator/attach_object")
    rospy.wait_for_service("/ipa_planning_scene_creator/detach_object")
    rospy.wait_for_service("/ipa_planning_scene_creator/remove_all_collision_objects")

    load_environment_client = rospy.ServiceProxy("/ipa_planning_scene_creator/load_environment", AddCollisionObject)
    unload_environment_client = rospy.ServiceProxy("/ipa_planning_scene_creator/unload_environment", RemoveCollisionObject)
    add_collision_object_client = rospy.ServiceProxy("/ipa_planning_scene_creator/add_collision_objects", AddCollisionObject)
    remove_collision_object_client = rospy.ServiceProxy("/ipa_planning_scene_creator/remove_collision_objects", RemoveCollisionObject)
    attach_client = rospy.ServiceProxy("/ipa_planning_scene_creator/attach_object", AddCollisionObject)
    deatch_client = rospy.ServiceProxy("/ipa_planning_scene_creator/detach_object", RemoveCollisionObject)
    #remove_all_collision_client = rospy.ServiceProxy("/ipa_planning_scene_creator/remove_all_collision_objects", Trigger)


    ##############################################################
    ### Test loading environment service
    object_pose = object_pose
    object_label = object_label
    id = object_id
    create_transformation_frame(child_frame_id=object_label+id, object_pose=object_pose)
    loadEnvironment(client=load_environment_client, label=object_label, id=id, object_pose=object_pose)
    # wait just for visualization of load and unload environment
    rospy.sleep(2.0)

    # test unload environment service
    unloadEnvironment(client=unload_environment_client, label=object_label, id=id)

    ############################################################
    ### Test collision object service
    object_pose = object_pose
    object_label = object_label
    id = object_id
    create_transformation_frame(child_frame_id=object_label+id, object_pose=object_pose)
    addCollisionObject(client=add_collision_object_client, label=object_label, id=id, object_pose=object_pose)

    # wait just for visualization of load and unload environment
    rospy.sleep(2.0)

    # test remove collision object service
    removeCollisionObject(client=remove_collision_object_client, label=object_label, id=id)

    ############################################################
    ### Test collision object service
    object_pose = object_pose
    object_label = object_label
    id = object_id
    #create_transformation_frame(child_frame_id=object_label+id, object_pose=object_pose)
    attachObject(client=attach_client, label=object_label, id=id, object_pose=object_pose)

    # wait just for visualization of load and unload environment
    rospy.sleep(2.0)

    # test remove collision object service
    detachObject(client=deatch_client, label=object_label, id=id)

    #removeAllCollisionObject(label='', id='')

def mainGripperHandler(finger_value=0.01, finger_open=False, finger_close=True):
    '''
        Gripper handler function use to control gripper open/colse command,
        move parallel part of gripper with give values (in cm)
        @param finger_value is the big parallel part move realtive to zero position
        @finger_open open the small finger
        @finger_close close the small finger
    '''

    rospy.wait_for_service("/gripper/driver/set_object")
    small_finger_client = rospy.ServiceProxy("/gripper/driver/set_object", SetObject)

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

    response = small_finger_client.call(request)
    if response.success:
        rospy.loginfo(response.message)
        return response.success
    else:
        rospy.logerr(response.message)
        rospy.logerr("SetObject service call Failed!!")
        return False

    '''
    if small_finger_client.call(request) is True:
        response = SetObjectResponse()
        rospy.loginfo(response.message)
        return response.success
    else:
        response = SetObjectResponse()
        rospy.logerr(response.message)
        rospy.logerr("SetObject service call Failed!!")
        return False
    '''


'''
if __name__ == '__main__':
    rospy.init_node("test_arm_planning")

    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', required=True, default='raw3-1', help='robot used loading planning queries')
    parser.add_argument('--base_link', required=True, default='base_link', help='base link of the kinematic chain')
    parser.add_argument('--tip_link', required=True, default='gripper',
                        help='tip link of kinematic chain used for attaching object')

    args, unknown = parser.parse_known_args()

    # Testing to gripper service
    mainGripperHandler(finger_open=False, finger_close=True)
    rospy.spin()

    # rospack path
    rospack = rospkg.RosPack()

    # rospack.list() # list all packages, equivalent to ropack list
    path = rospack.get_path('ipa_arm_planning')

    # load yaml file for shelf quires, assume that planning queries load from launch file
    # loader = yaml.load(open('/home/mjp/telekom/src/ipa_manipulation/ipa_arm_planning/planning_queries/'+'schunk_lwa4p_extended_shelf_query_list.yaml'))
    loader = rosparam.load_file((path + '/planning_queries/' + args.robot + '_shelf_query_list.yaml'))
    for params, ns in loader:
        rosparam.upload_params(ns=ns, values=params, verbose=False)

    # read planning quiries from paramter server
    objects_list = rospy.get_param('/queries')

    # tf broadcaster, publishing object frame
    #publish_transform_msg(param_name="/queries")

    ns_global_prefix = rospy.get_namespace()
    rospy.loginfo("Node namespace: %s",ns_global_prefix)

    ik_param_string = '/arm' + "/kinematic_calculation/" + "ik_topic_name"
    if not rospy.has_param(ik_param_string):
        rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting ...", ik_param_string)
    ik_srv_name = rospy.get_param(ik_param_string)

    plan_client = actionlib.SimpleActionClient('/arm_planning_node/PlanTo', ipa_manipulation_msgs.msg.PlanToAction)
    plan_client.wait_for_server(rospy.Duration(5.0))
    execution_client = actionlib.SimpleActionClient('/traj_exec', ExecuteTrajectoryAction)
    execution_client.wait_for_server(timeout=rospy.Duration(5.0))

    for name, data in objects_list.items():
        if rospy.is_shutdown():
            break
        print("Executing task on target: " + name)

        # start_state = make_robot_state(data['start_state']['name'],
        #                               data['start_state']['position'])
        start_state = data['start_state']['position']

        target_pose = make_pose(data['goal_frame']['position'],
                                data['goal_frame']['orientation'],
                                data['goal_frame']['frame_id'])

        # base_frame = data['goal_frame']['frame_id']

        # planning scene handler function
        mainPlanningScene(object_label='box', object_id='0', object_pose=target_pose)
        rospy.spin()

        print "!!!!!!!!!!!!!!!!!! point 1 !!!!!!!!!!!!!!!"
        target_joint_values, ik_success = ikService(
            service_name= "/arm/kinematic_calculation/computeIK", #'/arm/computeIK',
            base_link='world',
            joint_values = rospy.wait_for_message("/" + "arm" + "/joint_states", JointState, timeout = 3.0).position,
            eef_pose= target_pose #'arm_base_link'
        )
        print "=============================="
        print target_joint_values
        print "=============================="
#        # plan trajectory
#        trajecotry, valid = planTrajectoryInJointSpace(client=plan_client, bdmp_goal=None, bdmp_action='', object_pose=start_state)
#        #executionTrajectory(follow_plan=False, object_pose=PoseStamped())
#        print trajecotry
#        confirm = raw_input("confirm: ")
#        executionActionServer(execution_client, trajectory=trajecotry)

        trajecotry, valid = planTrajectoryInCartSpace(client=plan_client, bdmp_goal=None, bdmp_action='', object_pose=target_pose)
        if valid is False and ik_success is True:
            trajecotry, valid = planTrajectoryInJointSpace(bdmp_goal=None, bdmp_action='', object_pose=target_joint_values)
        #executionTrajectory(follow_plan=False, object_pose=PoseStamped())
        confirm = raw_input("confirm: ")
        executionActionServer(execution_client, trajectory=trajecotry)


        #object_pose = make_pose(position=[0.05, 0., 0.], orienatation=[0., 0., 0., 1.0], frame_id=args.tip_link)
        #cartesianPlan(bdmp_goal=None, bdmp_action='', object_pose=object_pose)
        #cartesianExecution(follow_plan=False,
        #                         object_pose=object_pose)

        rospy.sleep(2.0)
        #rospy.spin()
    rospy.spin()
'''
