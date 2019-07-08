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

            detection.pose.header.frame_id = 'base_link';
            detection.pose.pose.position.x = round(random.random()*0.02, 2)
            detection.pose.pose.position.y = round(random.random()*0.02 - 0.01, 2)

            detections = DetectionArray()
            detections.detections = [detection]

            self.publisher_.publish(detections)

    def handleStartService(self, request):
        print('[Service {}] Starting Detection'.format(self.name_))
        self.mutex_.acquire()
        if self.is_running_:
            self.mutex_.release()
            return TriggerResponse()

        self.mutex_.release()
        self.is_running_ = True
        Thread(target=self.talker).start()
        return TriggerResponse()

    def handleStopService(self, request):
        print('[Service {}] Stopping Detection'.format(self.name_))
        self.mutex_.acquire()
        self.is_running_ = False
        self.mutex_.release()
        return TriggerResponse()

if __name__ == "__main__":
    try:
        rospy.init_node('fake_trash_dirt_detector', anonymous=True)

        args = rospy.myargv(argv=sys.argv)
        if '--dirt' in args:
            dirt_detector = Detector('dirt_detection_client_preprocessing', '/dirt_detection_client_preprocessing/dirt_detector_topic', 5000)
        if '--trash' in args:
            trash_detector = Detector('trash_detector', 'trash_detector_topic', 1)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
