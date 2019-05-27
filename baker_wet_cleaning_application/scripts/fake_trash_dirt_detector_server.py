#!/usr/bin/env python

import rospy
from cob_object_detection_msgs.msg import DetectionArray, Detection
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Empty, EmptyResponse

from threading import Thread, Lock

import random

class Detector:

    def __init__(self, name, frequency=0.2):
        self.name_ = name
        self.mutex_ = Lock()
        self.rate_ = frequency
        random.seed(0) # todo (rmb-ma): deterministic random (not really because of time)

        rospy.Service(self.name_ + '/start_detection', Empty, self.handleStartService)
        rospy.Service(self.name_ + '/stop_detection', Empty, self.handleStopService)

        self.is_running_ = False
        self.publisher_ = rospy.Publisher(self.name_ + '_topic', DetectionArray, queue_size=10)

        self.current_position_ = (0, 0)

    def updateCurrentPosition(self, message):
        translation = message.transforms[0].transform.translation
        x = translation.x
        y = translation.y
        self.mutex_.acquire()
        self.current_position_ = (x, y)
        self.mutex_.release()

    def talker(self):
        rospy.Subscriber("/tf", TFMessage, self.updateCurrentPosition)

        rate = rospy.Rate(self.rate_) # 0.2Hz
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

            self.mutex_.acquire()
            detection.pose.pose.position.x = self.current_position_[0] + random.random()*0.02 - 0.01
            detection.pose.pose.position.y = self.current_position_[1] + random.random()*0.02 - 0.01
            self.mutex_.release()

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
        dirt_detector = Detector('dirt_detector', 0.000000001)
        #dirt_detector.handleStartService(0)
        trash_detector = Detector('trash_detector', 0.00000000001)
        #trash_detector.handleStartService(0)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
