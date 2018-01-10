#!/usr/bin/env python

import roslib
roslib.load_manifest('baker_office_cleaning_application')
import actionlib
import rospy
import tf
import numpy as np
import cv2
import sys

#import baker_office_cleaning_application.scripts

#move_base_msgs
from baker_msgs.srv import *
from ipa_building_msgs.msg import *
#from ipa_building_msgs.action import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose2D, Point32, Quaternion
from sensor_msgs.msg import Image
from move_base_msgs.msg import *
from scitos_msgs.msg import MoveBasePathAction
from scitos_msgs.msg import MoveBasePathGoal
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
tmp_map_bridge
#import simple_script_server
#sss = simple_script_server.simple_script_server()

class TestClass():
	def explore(self):
		print "TestClass WORKS"

#class SeqControl(simple_script_server.script):
class SeqControl():
	def explore(self):

		# receive the navigation map in sensor_msgs/Image format
		print "Waiting for service '/baker/get_map_image' to become available ..."
		rospy.wait_for_service('/baker/get_map_image')
		try:
			get_map = rospy.ServiceProxy('/baker/get_map_image', GetMap)
			self.map_data = get_map()
		except rospy.ServiceException, e:
			print "Map-Receiving-Service call failed: %s" % e
		print "Map received with resolution: ", self.map_data.map_resolution, "   and origin: ", self.map_data.map_origin


		# compute map division into rooms (ipa_room_segmentation)
		segmentation_goal = MapSegmentationGoal()
		segmentation_goal.input_map = self.map_data.map  
		rospy.init_node('exploration_node')# todo: use the current room map
		segmentation_goal.map_resolution = self.map_data.map_resolution
		segmentation_goal.map_origin = self.map_data.map_origin
		segmentation_goal.return_format_in_meter = False
		segmentation_goal.return_format_in_pixel = True
		segmentation_goal.robot_radius = 0.3
		print "Waiting for Action '/room_segmentation/room_segmentation_server' to become available..."
		segmentation_client = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server', MapSegmentationAction)
		segmentation_client.wait_for_server()
		print "Sending goal..."
		segmentation_client.send_goal(segmentation_goal)
		segmentation_client.wait_for_result()
		segmentation_result = segmentation_client.get_result()
		print "Segmentation completed"

		
		# compute optimal room visiting order (ipa_building_navigation)
		room_sequence_goal = FindRoomSequenceWithCheckpointsGoal()
		room_sequence_goal.input_map = self.map_data.map
		room_sequence_goal.map_resolution = self.map_data.map_resolution
		room_sequence_goal.map_origin = self.map_data.map_origin
		room_sequence_goal.robot_radius = 0.3
		room_sequence_goal.room_information_in_pixel = segmentation_result.room_information_in_pixel
		room_sequence_goal.robot_start_coordinate.position = Point32(x=8., y=14.)  # actual current coordinates should be inserted
		room_sequence_goal.robot_start_coordinate.orientation = Quaternion(x=0.,y=0.,z=0., w=0.)
		print "Waiting for Action '/room_sequence_planning/room_sequence_planning_server' to become available..."
		room_sequence_client = actionlib.SimpleActionClient('/room_sequence_planning/room_sequence_planning_server', FindRoomSequenceWithCheckpointsAction)
		room_sequence_client.wait_for_server()
		print "Sending goal..."
		room_sequence_client.send_goal(room_sequence_goal)
		room_sequence_client.wait_for_result()
		room_sequence_result = room_sequence_client.get_result()
		print "Room Sequencing completed"
			

		# compute exploration path
		planning_mode = 2 # viewpoint planning
		fov_points = [Point32(x=0.04035, y=0.136), Point32(x=0.04035, y=-0.364), Point32(x=0.54035, y=-0.364), Point32(x=0.54035, y=0.136)] # this field of view represents the off-center iMop floor wiping device
		#fov_points = [Point32(x=0.15, y=0.35), Point32(x=0.15, y=-0.35), Point32(x=1.15, y=-0.65), Point32(x=1.15, y=0.65)] # this field of view fits a Asus Xtion sensor mounted at 0.63m height (camera center) pointing downwards to the ground in a respective angle
		# ... or ...
		#planning_mode = 1 # footprint planning
		#fov_points = [Point32(x=-0.3, y=0.3), Point32(x=-0.3, y=-0.3), Point32(x=0.3, y=-0.3), Point32(x=0.3, y=0.3)] # this is the working area of a vacuum cleaner with 60 cm width
		

		# convert cv_bridge map to opencv image
		self.bridge = CvBridge()
		map_opencv = self.bridge.imgmsg_to_cv2(segmentation_result.segmented_map, desired_encoding = "passthrough")

		# loop over detected rooms in optimal order
		for current_room in range(1, len(room_sequence_result.checkpoints)):

			# move to the room center of the current room
			current_room_coordinates = Point32(x=-100, y=-100, z=-100)
			print "Moving to room %i ..." % current_room
			move_base_goal = MoveBaseGoal()
			move_base_goal.target_pose.pose.position = Point32(x=0, y=0, z=0)
			move_base_goal.target_pose.pose.orientation = Quaternion(x=0., y=0., z=0., w=0.)
			move_base_goal.target_pose.header.frame_id = 'base_link'
			move_base_goal.target_pose.header.stamp = rospy.Time.now()
			print "Waiting for Action 'move_base' to become available..."
			move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
			move_base_client.wait_for_server()
			print "Sending goal..."
			move_base_client.send_goal(move_base_goal)
			move_base_client.wait_for_result()
			move_base_result = move_base_client.get_result()
			

			# create room map (255=room, 0=walls, other rooms)
			print "Creating room map..."
			image_height, image_width = map_opencv.shape
			tmp_map_opencv = np.zeros((image_width, image_height), np.uint8)
			current_room_index = room_sequence_result.checkpoints[current_room].room_indices
			for x in range(image_width):
				for y in range(image_height):
					if (map_opencv[y, x] == current_room_index):
						tmp_map_opencv[y, x] = 255
						
			tmp_map_bridge = self.bridge.cv2_to_imgmsg(tmp_map_opencv, encoding = "mono8")
			print "Room map completed"

			# explore the current room
			exploration_goal = RoomExplorationGoal()
			exploration_goal.input_map = tmp_map_bridge
			exploration_goal.map_resolution = self.map_data.map_resolution
			exploration_goal.map_origin = self.map_data.map_origin
			exploration_goal.robot_radius = 0.3
			exploration_goal.coverage_radius = 0.3
			exploration_goal.field_of_view = fov_points
			exploration_goal.starting_position = Pose2D(x=1., y=0., theta=0.)
			exploration_goal.planning_mode = planning_mode
			print "Waiting for action '/room_exploration/room_exploration_server' to become available ..."
			exploration_client = actionlib.SimpleActionClient('/room_exploration/room_exploration_server', RoomExplorationAction)
			exploration_client.wait_for_server()
			print "Sending goal ..."
			exploration_client.send_goal(exploration_goal)
			exploration_client.wait_for_result()
			exploration_result = exploration_client.get_result()
			print "Exploration path received with length ", len(exploration_result.coverage_path_pose_stamped)

			# command path follow movement
			move_base_path_goal = MoveBasePathGoal()
			move_base_path_goal.target_poses = exploration_result.coverage_path_pose_stamped
			move_base_path_goal.path_tolerance = 0.2 #0.1
			move_base_path_goal.goal_position_tolerance = 0.5 #0.25 #0.1
			move_base_path_goal.goal_angle_tolerance = 1.57 #0.7 #0.087
			print "Waiting for action 'move_base_path' to become available ..."
			move_base_path = actionlib.SimpleActionClient('/move_base_path', MoveBasePathAction)
			move_base_path.wait_for_server()
			print "Sending goal ..."
			move_base_path.send_goal(move_base_path_goal)
			move_base_path.wait_for_result()
			print move_base_path.get_result()
		
			# command wall following movement
			move_base_path_goal = MoveBasePathGoal()
			move_base_path_goal.target_poses = exploration_result.coverage_path_pose_stamped
			move_base_path_goal.path_tolerance = 0.2 #0.1
			move_base_path_goal.goal_position_tolerance = 0.4 #0.25 #0.1
			move_base_path_goal.goal_angle_tolerance = 3.14 #0.7 #0.087
			print "Waiting for action 'move_base_path' to become available ..."
			move_base_path = actionlib.SimpleActionClient('/move_base_wall_follow', MoveBasePathAction)
			move_base_path.wait_for_server()
			print "Sending goal ..."
			move_base_path.send_goal(move_base_path_goal)
			move_base_path.wait_for_result()
			print move_base_path.get_result()


if __name__ == '__main__':
	try:
		rospy.init_node('exploration_node')
		command = 'script_1 = TestClass()'
		exec(command)
		script_1.explore()

		script = SeqControl()
		script.explore()
	except rospy.ROSInterruptException:
		print "Keyboard Interrupt"
