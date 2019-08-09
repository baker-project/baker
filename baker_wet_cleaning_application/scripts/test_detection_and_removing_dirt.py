#!/usr/bin/env python

from threading import Thread
import services_params as srv
from std_srvs.srv import Trigger
from cob_object_detection_msgs.msg import DetectionArray
import rospy
from utils import projectToCamera
from dirt_removing_behavior import DirtRemovingBehavior

"""
This file contains routine to start the dirt detection on the current
robot location and then execute the dirt removing behavior if needed.

WARNING: the functions used in this code are partly copy pasted from DryCleaningBehavior
and could be outdated (7.08.2019)

Usage:
 1/ start the baker_wet_cleaning_application
 2/ Start this file

"""

class TestDetector:

    def __init__(self):
        self.trash_topic_subscriber_ = rospy.Subscriber('/dirt_detection_server_preprocessing/dirt_detector_topic', DetectionArray, self.dirtDetectionCallback)
        self.callTriggerService(srv.START_DIRT_DETECTOR_SERVICE_STR)

    def callTriggerService(self, service_name):
        rospy.wait_for_service(service_name)
        try:
			req = rospy.ServiceProxy(service_name, Trigger)
			req()
        except rospy.ServiceException, e:
			print("Service call to {} failed: {}".format(service_name, e))

    def dirtDetectionCallback(self, detections):
        print('dirts detected')
        self.callTriggerService(srv.STOP_DIRT_DETECTOR_SERVICE_STR)

        detections = detections.detections

		# todo rmb-ma temporary solution. Keep camera, robot or room coordinates?
        print('Project positions to camera')
        detections = [projectToCamera(detection) for detection in detections]

        if None in detections or len(detections) == 0:
            print('No detections can be projected to the camera. Startin detection again')
            self.callTriggerService(srv.START_DIRT_DETECTOR_SERVICE_STR)
            return

        print("DIRT(S) DETECTED!!")

		# 1. Stop the dirt and the trash detections

        # 2. Clean them
        for detection in detections:
    		dirt_remover = DirtRemovingBehavior("DirtRemovingBehavior", [0],
    											move_base_service_str=srv.MOVE_BASE_SERVICE_STR,
    											map_accessibility_service_str=srv.MAP_ACCESSIBILITY_SERVICE_STR,
    											clean_pattern_str=srv.CLEAN_PATTERN_STR)

    		position = detection.pose.pose.position
    		dirt_remover.setParameters(dirt_position=position)
    		dirt_remover.executeBehavior()


if __name__ == '__main__':
    rospy.init_node('test_dirt_detector', anonymous=True)
    TestDetector()
    rospy.spin()
