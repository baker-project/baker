#!/usr/bin/env python

import rospy
import tf

"""
Tf publisher to publish the position of the vacuum cleaner
"""

if __name__ == '__main__':
    rospy.init_node('tf_arm_publisher')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # The vacuum cleaner is 40 cm in front of the robot
        br.sendTransform(translation=(0.40, 0, 0),
                         rotation=(0.0, 0.0, 0.0, 1.0),
                         time=rospy.Time.now(),
                         child="vacuum_cleaner",
                         parent="base_link")

        try:
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException, e:
            rospy.logwarn("ROSTimeMovedBackwardsException during sleep(). Continue anyway...")
        except rospy.exceptions.ROSInterruptException as e:
            pass
