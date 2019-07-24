#!/usr/bin/env python

import rospy
from cob_object_detection_msgs.msg import DetectionArray, Detection
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, TriggerResponse
from tf_converter import projectToFrame


class TrashcanVisualizer:

    def __init__(self, detection_topic, visualizer_topic):
        self.detection_topic_ = detection_topic
        self.visualizer_topic_ = visualizer_topic
        self.lifetime_ = 50
        rospy.Subscriber(detection_topic, DetectionArray,self.talker)
        self.publisher_ = rospy.Publisher(visualizer_topic, MarkerArray, queue_size=10)

    def talker(self, detections):
        print("talking talking")
        markers = MarkerArray()
        for detection in detections.detections:
            print(detection)
            pose = detection.pose
            pose_in_map = projectToFrame(pose, targeted_frame='map')

            marker = Marker()
            # Publish a base
            for j in range(3):
                marker.header.frame_id = 'map'
                marker.ns = 'trashcan_detection'
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.color.a = 0.85
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 0
                marker.lifetime = rospy.Duration(self.lifetime_)
                marker.scale.x = 0.01
                marker.scale.y = 0.015
                marker.scale.z = 0

                point_A = Point()
                point_A.x = 0.
                point_A.y = 0.
                point_A.z = 0

                point_B = Point()
                point_B.x = 0
                point_B.y = 0
                point_B.z = 0
                marker.points = [point_A, point_B]
                if j == 0:
                    marker.points[1].x = 0.2
                    marker.color.r = 255
                if j == 1:
                    marker.points[1].y = 0.2
                    marker.color.g = 255
                if j == 2:
                    marker.points[1].z = 0.2
                    marker.color.b = 255

                markers.markers.append(marker)

            marker.header.frame_id = 'map'
            marker.ns = "trashcan_detection"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.a = 0.9
            marker.color.r = 255
            marker.color.g = 153
            marker.color.b = 51

            marker.pose = pose_in_map.pose

            marker.lifetime = rospy.Duration(self.lifetime_)
            marker.scale.x = detection.bounding_box_lwh.x
            marker.scale.y = detection.bounding_box_lwh.y
            marker.scale.z = detection.bounding_box_lwh.z
            markers.markers.append(marker)

        self.publisher_.publish(markers)

if __name__ == "__main__":
    try:
        rospy.init_node('trashcan_visualizer', anonymous=True)
        TrashcanVisualizer(detection_topic='/trash_detector_topic', visualizer_topic='/trashcan_visualization_topic')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
