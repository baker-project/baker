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


def getCameraPosition(time=None):
	if time is None:
		time = rospy.Time.now()
	else:
		print(time)

	try:
		listener = getTransformListener()
		listener.waitForTransform('/map', '/camera1_optical_frame', time, rospy.Duration(10))
		(camera_pose_translation, camera_pose_rotation) = listener.lookupTransform('/map', '/base_link', time)

	except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,), e:
		print "Could not lookup robot pose: %s" % e
		return None, None, None

	camera_pose_rotation_euler = tf.transformations.euler_from_quaternion(camera_pose_rotation, 'rzyx')
	return camera_pose_translation, camera_pose_rotation, camera_pose_rotation_euler


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
