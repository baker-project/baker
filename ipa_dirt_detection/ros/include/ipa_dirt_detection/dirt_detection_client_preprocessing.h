#pragma once

//STANDARD
#include <iostream>
#include <math.h>
#include <stdio.h>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>

//MESSAGES
#include <baker_msgs/DirtDetectionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_object_detection_msgs/Detection.h>
#include <geometry_msgs/Pose.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ipa_dirt_detection/DirtDetectionPreprocessingConfig.h>

//TOPICS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

//OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//TF
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

//TIMER
#include <time.h>

//todo: just for saving delete it
//	<!-- send parameters to parameter server -->
//	<!-- <rosparam command="load" ns="dirt_detection/dirt_detection" file="$(find ipa_dirt_detection)/ros/launch/dirt_detection_client_preprocessing_params.yaml"/>
//	<rosparam command="load" ns="dirt_detection/dirt_detection" file="$(find ipa_dirt_detection)/ros/launch/dirt_detection_server_spectral_params.yaml"/> -->

namespace IpaDirtDetectionPreprocessing
{ //starting namespace

class ClientPreprocessing
{ //starting class
public:
		//Constructor
		ClientPreprocessing(ros::NodeHandle node_handle);
		//Destructor.
		~ClientPreprocessing();

		//================ Functions ==================================================================================
		//=============================================================================================================

		/**
		 * Function is called if point cloud topic is received.
		 *
		 * @param [in] point_cloud2_rgb_msg	Point cloude message from camera.
		 *
		 */
		void preprocessingCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);

		/**
		 * Converts: "sensor_msgs::PointCloud2" \f$ \rightarrow \f$ "pcl::PointCloud<pcl::PointXYZRGB>::Ptr".
		 *
		 * The function detects a plane in the point cloud, creates a mask for the plane pixels and sets all pixels in
		 * "plane_color_image" to their color if they lie in the plane, else to black.
		 *
		 *	@param [in] 	input_cloud 				Point cloud for plane detection.
		 *	@param [out] 	plane_color_image 			Shows the true color of all pixel within the plane. The size of the image is determined with the help of the point cloud!
		 *	@param [out]	plane_mask					Mask to separate plane pixels. Plane pixels are white (255), all other pixels are black (0).
		 *	@return 		True if any plane could be found in the image.
		 */
		bool planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, const std_msgs::Header& header, cv::Mat& plane_color_image, cv::Mat& plane_mask,
				pcl::ModelCoefficients& plane_model, const tf::StampedTransform& transform_map_camera);

		/// remove perspective from image
		/// @param H Homography that maps points from the camera plane to the floor plane, i.e. pp = H*pc
		/// @param R Rotation matrix for transformation between floor plane and world coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
		/// @param t Translation vector. See Rotation matrix.
		/// @param cameraImagePlaneOffset Offset in the camera image plane. Conversion from floor plane to  [xc, yc]
		bool computeBirdsEyePerspective(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask,
				pcl::ModelCoefficients& plane_model, cv::Mat& H, cv::Mat& R, cv::Mat& t, cv::Point2f& cameraImagePlaneOffset, cv::Mat& plane_color_image_warped,
				cv::Mat& plane_mask_warped);

protected:
		/**
		 * services
		 */
		ros::ServiceServer activate_dirt_detection_service_server_; /// server for activating dirt detection
		ros::ServiceServer deactivate_dirt_detection_service_server_; /// server for deactivating dirt detection

		bool activateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

		bool deactivateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

		void dynamicReconfigureCallback(ipa_dirt_detection::DirtDetectionPreprocessingConfig &config, uint32_t level);

		/// dynamic reconfigure
		dynamic_reconfigure::Server<ipa_dirt_detection::DirtDetectionPreprocessingConfig> dynamic_reconfigure_server_;

		//ROS node handle
		ros::NodeHandle node_handle_;

		//Used to receive point cloud topic from camera.
		ros::Subscriber camera_depth_points_sub_;

		//Used to subscribe and publish images.
		image_transport::ImageTransport* it_;

		//Publisher for plane segmented image.
		ros::Publisher floor_plane_pub_;

		//Publisher for the detected dirt
		ros::Publisher dirt_detected_;

		bool dirt_detection_callback_active_; ///< flag whether incoming messages shall be processed

		actionlib::SimpleActionClient<baker_msgs::DirtDetectionAction> dirt_detection_client_;		// client and server need same name ipa_dirt_detection_action

		//PARAMS
		double bird_eye_resolution_; // resolution for bird eye's perspective [pixel/m]
		bool warp_image_; // if true, image warping to a bird's eye perspective is enabled todo: make this dynamic reconfigure or param (joel)
		bool use_mask_; //paramter to use mask or not. if not all points belong to plane.
		int detect_scales_;

		std::string bird_eye_resolution_string_;

		std::map<std::string, bool> debug_;

		//floor plane search
		int floor_search_iterations_; // the number of attempts to segment the floor plane in the image
		int min_plane_points_; // minimum number of points that are necessary to find the floor plane

		double max_distance_to_camera_; // only those points which are close enough to the camera are taken [max distance in m]

		//multiscale search
		double image_scaling_;
		double bird_eye_start_resolution_; // smallest resolution for perspective transform
		double bird_eye_base_resolution_; // base resolution of perspective transformation

		//For received result from action server: Means image with positions of dirt
		image_transport::Publisher dirt_detection_image_pub_; // topic for publishing the image containing the dirt positions

}; //ending class

}; //ending namespace
