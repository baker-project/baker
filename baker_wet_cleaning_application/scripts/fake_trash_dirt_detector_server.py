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
        rospy.init_node(name, anonymous=True)
        random.seed(0) # todo (rmb-ma): deterministic random (not really because of time)

        rospy.Service(self.name_ + '/start_detection', Empty, self.handleStartService)
        rospy.Service(self.name_ + '/stop_detection', Empty, self.handleStopService)

        self.is_running_ = False
        self.publisher_ = rospy.Publisher(self.name_ + '_topic', DetectionArray, queue_size=10)

        rospy.spin()


    def talker(self):
        print("talker is running? and rospy.isShutdown= {}".format(rospy.is_shutdown()))
        rate = rospy.Rate(20) # 0.2Hz
        while not rospy.is_shutdown():
            self.mutex_.acquire()
            print("in talker: interrupt_detection {}".format(self.is_running_))
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

            rospy.loginfo('DETECTION')
            self.publisher_.publish(detections)


    def handleStartService(self, request):
        self.mutex_.acquire()
        if self.is_running_:
            self.mutex_.release()
            return EmptyResponse()
        self.mutex_.release()
        print("Should start the detection")
        self.is_running_ = True
        Thread(target=self.talker).start()
        return EmptyResponse()


    def handleStopService(self, request):
        print("should stop the detection")
        self.mutex_.acquire()
        self.is_running_ = False
        self.mutex_.release()
        return EmptyResponse()

if __name__ == "__main__":
    try:
        dirt_detector = Detector('dirt_detector')
        trash_detector = Detector('trash_detector')

    except rospy.ROSInterruptException:
        pass
