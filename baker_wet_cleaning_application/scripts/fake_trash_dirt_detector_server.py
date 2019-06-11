#!/usr/bin/env python

import rospy
from cob_object_detection_msgs.msg import DetectionArray, Detection
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Empty, EmptyResponse

from threading import Thread, Lock

import random
from utils import getCurrentRobotPosition


class Detector:

    def __init__(self, name, frequency=0.2):
        self.name_ = name
        self.rate_ = frequency
        self.mutex_ = Lock()
        random.seed(0)  # todo (rmb-ma): deterministic random (not really because of time)

        rospy.Service(self.name_ + '/start_detection', Empty, self.handleStartService)
        rospy.Service(self.name_ + '/stop_detection', Empty, self.handleStopService)

        self.is_running_ = False
        self.publisher_ = rospy.Publisher(self.name_ + '_topic', DetectionArray, queue_size=10)

        self.current_position_ = (0, 0)

    def talker(self):

        rate = rospy.Rate(self.rate_)  # 0.2Hz
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

            (translation, _, _) = getCurrentRobotPosition()
            (x, y) = (translation[0], translation[1]) if translation is not None else (0, 0)

            detection.pose.pose.position.x = round(x + random.random()*0.02 - 0.01, 0)
            detection.pose.pose.position.y = round(y + random.random()*0.02 - 0.01, 0)

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
        dirt_detector = Detector('dirt_detector', 2)
        #dirt_detector.handleStartService(0)
        trash_detector = Detector('trash_detector', 2)
        #trash_detector.handleStartService(0)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
