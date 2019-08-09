#!/usr/bin/env python

import rospy
from cob_object_detection_msgs.msg import DetectionArray, Detection
import sys
from std_srvs.srv import Trigger, TriggerResponse
from threading import Thread, Lock
from math import atan2, cos, sqrt
import random

import tf

###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl = None
_tl_creation_lock = Lock()


def getTransformListener():
	global _tl
	with _tl_creation_lock:
		if _tl is None:
			_tl = tf.TransformListener(interpolate=True, cache_time=rospy.Duration(40.0))
		return _tl

def getCurrentRobotPosition():
	# read out current robot pose
	try:
		listener = getTransformListener()
		t = rospy.Time(0)
		listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
		(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
	except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,), e:
		print("Could not lookup robot pose: %s" % e)
		return None, None, None
	robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation,
																		 'rzyx')  # yields yaw, pitch, roll

	return robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler


class Detector:

    def __init__(self, name, topic, frequency=0.2, dirt_positions_candidates=[]):
        self.name_ = name
        self.rate_ = frequency
        self.mutex_ = Lock()
        random.seed(0)

        rospy.Service(self.name_ + '/activate_detection', Trigger, self.handleStartService)
        rospy.Service(self.name_ + '/deactivate_detection', Trigger, self.handleStopService)

        self.is_running_ = False
        self.publisher_ = rospy.Publisher(topic, DetectionArray, queue_size=10)

        self.current_position_ = (0, 0)

        self.use_random_detections_ = len(dirt_positions_candidates) == 0
        self.dirt_positions_candidates_ = dirt_positions_candidates

        self.camera_frame_id_ = 'map'

    def talker(self):
        rate = rospy.Rate(self.rate_) # 0.2Hz
        while not rospy.is_shutdown():
            self.mutex_.acquire()
            if not self.is_running_:
                self.mutex_.release()
                return
            self.mutex_.release()

            rate.sleep()
            if random.random() > 0.001:
                continue

            detections = self.randomDetections() if self.use_random_detections_ else self.fixedDetections()
            if len(detections.detections) == 0:
                continue
            self.publisher_.publish(detections)

    def createDetection(self, position, orientation=(0, 0, 0.92521152, -0.37945176), size=(0.50, 0.50, 0.60)):
        detection = Detection()

        detection.header.stamp = rospy.Time.now()
        detection.header.frame_id = self.camera_frame_id_
        detection.pose.header.stamp = rospy.Time.now()
        detection.pose.header.frame_id = self.camera_frame_id_
        detection.pose.pose.position.x = position[0]
        detection.pose.pose.position.y = position[1]
        detection.pose.pose.position.z = position[2]

        detection.pose.pose.orientation.x = orientation[0]
        detection.pose.pose.orientation.y = orientation[1]
        detection.pose.pose.orientation.z = orientation[2]
        detection.pose.pose.orientation.w = orientation[3]

        detection.bounding_box_lwh.x = size[0]
        detection.bounding_box_lwh.y = size[1]
        detection.bounding_box_lwh.z = size[2]
        return detection

    def randomDetections(self):
        detections = DetectionArray()
        detections.header.frame_id = self.camera_frame_id_

        detection = self.createDetection((round(2*random.random() - 1, 2), round(2*random.random() - 1, 2), 0.6))
        detections.detections = [detection]

        return detections

    def isVisible(self, possible_dirt_position, robot_position):
        # possible dirt position and robot position shoud be both in map coordinates
        (translation, orientation, rotation) = robot_position
        if None in robot_position:
            return False

        dx = possible_dirt_position[0] - translation[0]
        dy = possible_dirt_position[1] - translation[1]
	#print('possible dirt_position {}'.format(possible_dirt_position))
	#print('robot position {}'.format(translation))
	#print('dx {} dy {}'.format(dx, dy))
        distance_robot_detection = sqrt(dx**2 + dy**2)
        print('distance', distance_robot_detection)
        if distance_robot_detection < 0.8 or 1.5  < distance_robot_detection:
            return False

        theta_detection = atan2(dy, dx)
	#print('theta detection {}'.format(theta_detection))
        theta_robot = rotation[0]
 	#print('theta robot {}'.format(rotation[0]))
	#print('rotation robot {}'.format(rotation))
	cos_angle_robot_detection = cos(theta_detection - theta_robot)
        print('cos ', cos_angle_robot_detection)
        return cos_angle_robot_detection > 0.5

    def fixedDetections(self):
        robot_position = getCurrentRobotPosition()

        detections = DetectionArray()
        detections.header.frame_id = self.camera_frame_id_
        detections.detections = []
        for possible_dirt_position in self.dirt_positions_candidates_:
            if self.isVisible(possible_dirt_position, robot_position):
                detection = self.createDetection(possible_dirt_position)
                detections.detections.append(detection)
        return detections

    def handleStartService(self, request):
        print('[Service {}] Starting Detection'.format(self.name_))
        response = TriggerResponse()

        self.mutex_.acquire()
        if self.is_running_:
            self.mutex_.release()
            response.success = False
            return response

        self.mutex_.release()
        self.is_running_ = True
        Thread(target=self.talker).start()
        response.success = True
        return response

    def handleStopService(self, request):
        response =  TriggerResponse()
        print('[Service {}] Stopping Detection'.format(self.name_))
        self.mutex_.acquire()
        self.is_running_ = False
        self.mutex_.release()
        response.success = True
        return response

if __name__ == "__main__":
    try:
        args = rospy.myargv(argv=sys.argv)
	rospy.init_node('fake_trash_dirt_detector', anonymous=True)
	# [(-1.619, -1.295, 0)])        
	if '--dirt' in args:
            dirt_detector = Detector('dirt_detection_server_preprocessing', '/dirt_detection_server_preprocessing/dirt_detector_topic', 1000,
                dirt_positions_candidates=[(-6.246809806159, -1.33480333451, 0), (-6.4158, -2.92, 0), (-5.37, -0.5, 0), (-6.15, -6.21, 0)])
		
        if '--trash' in args:
            trash_detector = Detector('trash_detector', 'trash_detector_topic', 1)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

