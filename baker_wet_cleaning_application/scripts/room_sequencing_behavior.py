#!/usr/bin/env python

###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
import threading

import actionlib
import rospy
import tf
from geometry_msgs.msg import Point32, Quaternion
from ipa_building_msgs.msg import FindRoomSequenceWithCheckpointsAction, FindRoomSequenceWithCheckpointsGoal

from scripts.tests import behavior_container

_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
	global _tl
	with _tl_creation_lock:
		if _tl is None:
			_tl=tf.TransformListener(True, rospy.Duration(40.0))
		return _tl
###########################################################################

class RoomSequencingBehavior(behavior_container.BehaviorContainer):

	#========================================================================
	# Description:
	# Class which contains the behavior for sequencing a list of rooms
	#========================================================================

	def __init__(self, behavior_name, interrupt_var, service_str):
		super(RoomSequencingBehavior, self).__init__(behavior_name, interrupt_var)
		self.service_str_ = service_str

	# Method for setting parameters for the behavior
	#def setParameters(self, map_data, segmentation_data, robot_radius):
	def setParameters(self, database, room_information_in_pixel, robot_radius):
		self.database_ = database
		self.room_information_in_pixel_ = room_information_in_pixel
		self.robot_radius_ = robot_radius

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
		except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, ), e:
			print "Could not lookup robot pose: %s" % e
			return (None, None, None)
		robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx')  # yields yaw, pitch, roll
		
		return (robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler)

	# Implemented Behavior
	def executeCustomBehavior(self):

		room_sequence_goal = FindRoomSequenceWithCheckpointsGoal()
		room_sequence_goal.input_map = self.database_.global_map_data_.map_image_
		room_sequence_goal.map_resolution = self.database_.global_map_data_.map_resolution_
		room_sequence_goal.map_origin = self.database_.global_map_data_.map_origin_
		room_sequence_goal.robot_radius = self.robot_radius_
		room_sequence_goal.room_information_in_pixel = self.room_information_in_pixel_
		(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = self.currentRobotPose()
		if (robot_pose_translation!=None):
			room_sequence_goal.robot_start_coordinate.position = Point32(x=robot_pose_translation[0], y=robot_pose_translation[1])  # actual current coordinates should be inserted
		else:
			self.printMsg("Warning: tf lookup failed, taking (0,0) as robot_start_coordinate.")
			room_sequence_goal.robot_start_coordinate.position = Point32(x=0, y=0)
		room_sequence_goal.robot_start_coordinate.orientation = Quaternion(x=0., y=0., z=0., w=0.)	# todo: normalized quaternion
		room_sequence_client = actionlib.SimpleActionClient(str(self.service_str_), FindRoomSequenceWithCheckpointsAction)
		self.printMsg("Running sequencing action...")
		self.room_sequence_result_ = self.runAction(room_sequence_client, room_sequence_goal)['result']
		self.printMsg("Room sequencing completed.")