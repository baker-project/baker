#!/usr/bin/env python

import rospy
import tf
from threading import Lock
from datetime import datetime

_tl = None
_tl_creation_lock = Lock()

def getTransformListener():
	global _tl
	with _tl_creation_lock:
		if _tl is None:
			_tl = tf.TransformListener(interpolate=True, cache_time=rospy.Duration(40.))
		return _tl

def projectToFrame(pose, targeted_frame):
	time = pose.header.stamp
	frame_id = pose.header.frame_id
	try:
		listener = getTransformListener()
		print("listener got")
		listener.waitForTransform(targeted_frame, frame_id, time, rospy.Duration(5.))
		print("waited")
		pose = listener.transformPose(targeted_frame, pose)
		print("transformed")
		return pose
	except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,), e:
		print("Could not lookup robot pose: %s" % e)
		return None
