#!/usr/bin/env python

import rospy
from cob_object_detection_msgs.msg import DetectionArray, Detection
from std_srvs.srv import Empty, EmptyResponse

from threading import Thread, Lock

import random

class Detector:

    def __init__(self, name):
        self.name_ = name
        self.mutex_ = Lock()
        random.seed(0) # todo (rmb-ma): deterministic random (not really because of time)

        rospy.Service(self.name_ + '/start_detection', Empty, self.handleStartService)
        rospy.Service(self.name_ + '/stop_detection', Empty, self.handleStopService)

        self.is_running_ = False
        self.publisher_ = rospy.Publisher(self.name_ + '_topic', DetectionArray, queue_size=10)


    def talker(self):
        rate = rospy.Rate(5) # 0.2Hz
        while not rospy.is_shutdown():
            self.mutex_.acquire()
            if not self.is_running_:
                self.mutex_.release()
                return
            self.mutex_.release()

            rate.sleep()
            if random.random() > 0.1:
                continue

            detection = Detection()
            detections = DetectionArray()
            detections.detections = [detection]

            self.publisher_.publish(detections)


    def handleStartService(self, request):
        print('[Service {}] Starting Detection'.format(self.name_))
        self.mutex_.acquire()
        if self.is_running_:
            self.mutex_.release()
            return EmptyResponse()
        self.mutex_.release()
        self.is_running_ = True
        Thread(target=self.talker).start()
        return EmptyResponse()


    def handleStopService(self, request):
        print('[Service {}] Stopping Detection'.format(self.name_))
        self.mutex_.acquire()
        self.is_running_ = False
        self.mutex_.release()
        return EmptyResponse()

if __name__ == "__main__":
    try:
        rospy.init_node('fake_trash_dirt_detector', anonymous=True)
        dirt_detector = Detector('dirt_detector')
        trash_detector = Detector('trash_detector')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
