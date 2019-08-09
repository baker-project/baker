#!/usr/bin/env python

from threading import Thread
import services_params as srv
from std_srvs.srv import Trigger
from cob_object_detection_msgs.msg import DetectionArray
import rospy
from utils import projectToCamera
from dirt_removing_behavior import DirtRemovingBehavior
from move_base_behavior import MoveBaseBehavior
from geometry_msgs.msg import Pose2D, Pose, Quaternion
from tf.transformations import quaternion_from_euler
import services_params as srv

if __name__ == '__main__':
    rospy.init_node('test_bug', anonymous=True)
    mover = MoveBaseBehavior("MoveBaseBehavior", [0], srv.MOVE_BASE_SERVICE_STR)

    best_pose3d = Pose()
    best_pose3d.position.x = -5.61
    best_pose3d.position.y = -5.84
    best_pose3d.position.z = 0

    theta  = -0.93
    orientation = quaternion_from_euler(0, 0, -0.85)

    best_pose3d.orientation.x = orientation[0]
    best_pose3d.orientation.y = orientation[1]
    best_pose3d.orientation.z = orientation[2]
    best_pose3d.orientation.w = orientation[3]

    mover.setParameters(
        goal_position=best_pose3d.position,
        goal_orientation=best_pose3d.orientation,
        header_frame_id='map',
        goal_position_tolerance=0.1,
        goal_angle_tolerance=0.1
    )
    mover.executeBehavior()

"""
    <!--The translational part of the transform-->
    <X>-5.49648197174</X>
    <!--The translational part of the transform-->
    <Y>-5.77961158752</Y>
    <!--The orientation in degrees-->
    <Phi>-53.285074947166564</Phi>
"""
