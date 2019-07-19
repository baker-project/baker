#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse

class Camera:

    def __init__(self, topic, frequency=10):
        self.rate_ = frequency
        self.publisher_ = rospy.Publisher(topic, PointCloud2, queue_size=10)
        self.talker()

    def talker(self):
        print("Fake camera running")
        rate = rospy.Rate(self.rate_) # 0.2Hz
        while not rospy.is_shutdown():
            self.publisher_.publish(PointCloud2())

if __name__ == "__main__":
    try:
        rospy.init_node('fake_camera', anonymous=True)
        Camera('/camera/depth_registered/points')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
