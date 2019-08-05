#!/usr/bin/env python

import rospy
from cob_object_detection_msgs.msg import DetectionArray, Detection
import sys
from std_srvs.srv import Trigger, TriggerResponse

from threading import Thread, Lock

import random

class Detector:

    def __init__(self, name, topic, frequency=0.2):
        self.name_ = name
        self.rate_ = frequency
        self.mutex_ = Lock()
        random.seed(0)

        rospy.Service(self.name_ + '/activate_detection', Trigger, self.handleStartService)
        rospy.Service(self.name_ + '/deactivate_detection', Trigger, self.handleStopService)

        self.is_running_ = False
        self.publisher_ = rospy.Publisher(topic, DetectionArray, queue_size=10)

        self.current_position_ = (0, 0)

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

            detection = Detection()

            detection.header.stamp = rospy.Time.now()
            # detection.header.frame_id = 'camera2_optical_frame'
            detection.header.frame_id = 'base_link'
            detection.pose.header.stamp = rospy.Time.now()
            # detection.pose.header.frame_id = 'camera2_optical_frame'
            detection.pose.header.frame_id = 'base_link'
            detection.pose.pose.position.x = round(2*random.random() - 1, 2)
            detection.pose.pose.position.y = round(2*random.random() - 1, 2)
            detection.pose.pose.position.z = 0.6#round(random.random()*0.2 - 0.01, 2)

            detection.pose.pose.orientation.x = 0.
            detection.pose.pose.orientation.y = 0.
            detection.pose.pose.orientation.z = 0.92521152
            detection.pose.pose.orientation.w = -0.37945176

            detection.bounding_box_lwh.x = 0.50
            detection.bounding_box_lwh.y = 0.50
            detection.bounding_box_lwh.z = 0.60

            detections = DetectionArray()
            # detections.header.frame_id = 'camera2_optical_frame'
            detections.header.frame_id = 'base_link'
            detections.detections = [detection]

            self.publisher_.publish(detections)

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
        if '--dirt' in args:
            rospy.init_node('fake_dirt_detector', anonymous=True)
            dirt_detector = Detector('dirt_detection_server_preprocessing', '/dirt_detection_server_preprocessing/dirt_detector_topic', 1)
        if '--trash' in args:
            rospy.init_node('fake_trashcan_detector', anonymous=True)
            trash_detector = Detector('trash_detector', 'trash_detector_topic', 2000)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
