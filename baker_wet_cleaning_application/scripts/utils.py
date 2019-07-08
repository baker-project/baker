#!/usr/bin/env python

import rospy
import tf
from threading import Lock
from datetime import datetime

###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl = None
_tl_creation_lock = Lock()


def getTransformListener():
	global _tl
	with _tl_creation_lock:
		if _tl is None:
			_tl = tf.TransformListener(interpolate=True, cache_time=rospy.Duration(40.0))
		return _tl


def projectToCamera(detection):

	time = detection.header.stamp
	try:
		listener = getTransformListener()
		listener.waitForTransform('/map', detection.header.frame_id, time, rospy.Duration(10))
		detection.pose = listener.transformPose('/map', detection.pose)
		return detection
	except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,), e:
		print("Could not lookup robot pose: %s" % e)
		return None

# retrieves the current robot pose
def getCurrentRobotPosition():
	# read out current robot pose
	try:
		listener = getTransformListener()
		t = rospy.Time(0)
		listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
		(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
	except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,), e:
		print "Could not lookup robot pose: %s" % e
		return None, None, None
	robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation,
																		 'rzyx')  # yields yaw, pitch, roll

	return robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler
