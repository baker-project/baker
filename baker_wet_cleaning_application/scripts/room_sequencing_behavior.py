#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose2D, Point32, Quaternion
from ipa_building_msgs.msg import *

import behavior_container


###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
import threading
import tf
_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
	global _tl
	with _tl_creation_lock:
		if _tl==None:
			_tl=tf.TransformListener(True, rospy.Duration(40.0))
		return _tl
###########################################################################

class RoomSequencingBehavior(behavior_container.BehaviorContainer):

	def __init__(self, interrupt_var_, service_str_):
		self.interrupt_var = interrupt_var_
		self.service_str = service_str_

	# Method for setting parameters for the behavior
	def setParameters(self, map_data_, segmentation_data_):
		self.map_data = map_data_
		self.segmentation_data = segmentation_data_

	# Method for returning to the standard pose of the robot
	def returnToRobotStandardState(self):
		# no data to save
		# nothing to be undone
		pass

	# retrieves the current robot pose
	def currentRobotPose(self):
		# read out current robot pose
		try:
			listener = get_transform_listener()
			t = rospy.Time(0)
			listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
			(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print "Could not lookup robot pose: %s" % e
			return (None, None, None)
		robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx')  # yields yaw, pitch, roll
		
		return (robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler)

	# Implemented Behavior
	def executeCustomBehavior(self):
		room_sequence_goal = FindRoomSequenceWithCheckpointsGoal()
		room_sequence_goal.input_map = self.map_data.map
		room_sequence_goal.map_resolution = self.map_data.map_resolution
		room_sequence_goal.map_origin = self.map_data.map_origin
		room_sequence_goal.robot_radius = 0.3
		room_sequence_goal.room_information_in_pixel = self.segmentation_data.room_information_in_pixel
		(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = self.currentRobotPose()
		room_sequence_goal.robot_start_coordinate.position = Point32(x=robot_pose_translation[0], y=robot_pose_translation[1])  # actual current coordinates should be inserted
		room_sequence_goal.robot_start_coordinate.orientation = Quaternion(x=0.,y=0.,z=0., w=0.)
		room_sequence_client = actionlib.SimpleActionClient(str(self.service_str), FindRoomSequenceWithCheckpointsAction)
		self.printMsg("Running sequencing action...")
		self.room_sequence_result = self.runAction(room_sequence_client, room_sequence_goal)
		self.printMsg("Room sequencing completed.")