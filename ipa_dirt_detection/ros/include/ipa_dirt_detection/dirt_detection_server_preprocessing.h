#pragma once

//STANDARD
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <vector>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>

//MESSAGES
#include <baker_msgs/DirtDetectionAction.h>
#include <baker_msgs/RotatedRect.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
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
				pcl::ModelCoefficients& plane_model, const float plane_inlier_threshold, const tf::StampedTransform& transform_map_camera);

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

		//For received result from action server: Means image with positions of dirt
		image_transport::Publisher dirt_detection_image_pub_; // topic for publishing the image containing the dirt positions

		bool dirt_detection_callback_active_; ///< flag whether incoming messages shall be processed
		double bird_eye_resolution_;          // resolution for bird eye's perspective [pixel/m]
		std::string bird_eye_resolution_string_;
		double image_scaling_;                // multiscale search

		actionlib::SimpleActionClient<baker_msgs::DirtDetectionAction> dirt_detection_client_;		// client and server need same name ipa_dirt_detection_action

		// parameters
		// ### scales and resolution for pre-processing
		int number_detect_scales_;     // # number of considered scales: single scale = 1, multi scale > 1
		double bird_eye_start_resolution_;    // smallest resolution for perspective transform, if multiple scales are checked, in [pixel/m]
		double bird_eye_base_resolution_;     // bird eye base/standard/reference resolution for bird eye's perspective transform, in [pixel/m]

		// ### image mask derived from floor plane points (masking can be deactivated by setting inlier_distance to a large value)
		std::vector<float> floor_plane_model_;    // floor plane parameters [a,b,c,d] for plane equation (a*x + b*y + c*z + d = 0), the plane defines the image mask,
		                                          // can be used for checking that the detected plane is similar to the target plane or to directly require this model without plane detection in the point cloud data
		                                          // float[] with 4 numbers a, b, c, d
		int floor_search_iterations_;     // the number of attempts to segment the floor plane in the image
		                                  // if set to 0 the parameters of floor_plane_model will be used diectly as plane model without fitting a plane into the 3d data
		float floor_plane_inlier_distance_;     // accepted distance of 3d points to the plane model to be considered inliers, in [m]
		int min_plane_points_;            // minimum number of points that are necessary to find the floor plane
		double max_distance_to_camera_;   // only those points which are close enough to the camera are taken, [max distance in m]

		// ### image perspective normalization
		bool is_warp_image_bird_perspective_enabled_; // if true, image warping to a bird's eye perspective is enabled todo: make this dynamic reconfigure or param (joel)

		// ### debug
		std::map<std::string, bool> debug_;

		//floor plane search




}; //ending class

}; //ending namespace
