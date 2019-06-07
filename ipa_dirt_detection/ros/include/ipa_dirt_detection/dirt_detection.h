/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: care-o-bot
 * \note
 * ROS stack name: baker
 * \note
 * ROS package name: ipa_dirt_detection
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: October 2011
 *
 * \brief
 * Module for detecting dirt on surfaces.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef DIRT_DETECTION_H_
#define DIRT_DETECTION_H_

//##################
//#### includes ####

// standard includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <time.h>
#include <math.h>
#include <sstream>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ipa_dirt_detection/DirtDetectionConfig.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// services
#include <std_srvs/Trigger.h>

// topics
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//bridge
#include <cv_bridge/cv_bridge.h>

//boost
#include <boost/thread/mutex.hpp>

#include <time.h>

namespace ipa_DirtDetection
{

/**
 *  Detects dirt from color image.
 */
class DirtDetection
{
protected:

	/**
	 * ROS node handle.
	 */
	ros::NodeHandle node_handle_;

	/// dynamic reconfigure
	dynamic_reconfigure::Server<ipa_dirt_detection::DirtDetectionConfig> dynamic_reconfigure_server_;

	/**
	 * services
	 */
	ros::ServiceServer activate_dirt_detection_service_server_; /// server for activating dirt detection
	ros::ServiceServer deactivate_dirt_detection_service_server_; /// server for deactivating dirt detection

	bool activateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	bool deactivateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	/**
	 * Used to subscribe and publish images.
	 */
	image_transport::ImageTransport* it_;

	tf::TransformListener transform_listener_;
	tf::TransformBroadcaster transform_broadcaster_;

	/**
	 * Used to receive color image topic from camera.
	 */
	image_transport::Subscriber color_camera_image_sub_;
	/**
	 * Used to receive point cloud topic from camera.
	 */
	ros::Subscriber camera_depth_points_sub_;

	ros::Publisher floor_plane_pub_;
	image_transport::Publisher dirt_detection_image_pub_; ///< topic for publishing the image containing the dirt positions

	//parameters
	int spectral_residual_gaussian_blur_iterations_;
	double dirt_threshold_;
	double spectral_residual_normalization_highest_max_value_;
	double spectral_residual_image_size_ratio_;
	double dirt_check_std_dev_factor_;
	double bird_eye_resolution_; // resolution for bird eye's perspective [pixel/m]
	bool dirt_detection_activated_on_startup_; // for normal operation mode, specifies whether dirt detection is on right from the beginning
	std::string dirt_mapping_mask_filename_; // if not an empty string, this enables using a mask that defines areas in the map where dirt detections are valid (i.e. this mask can be used to exclude areas from dirt mapping, white=detection area, black=do not detect)

	std::string birdeyeresolution;

	std::map<std::string, bool> debug_;

	bool warp_image_; // if true, image warping to a bird's eye perspective is enabled
	double max_distance_to_camera_; // only those points which are close enough to the camera are taken [max distance in m]
	bool remove_lines_; // if true, strong lines in the image will not produce dirt responses

	// plane search
	int floor_search_iterations_; // the number of attempts to segment the floor plane in the image
	int min_plane_points_; // minimum number of points that are necessary to find the floor plane
	double plane_normal_max_z_; // maximum z-value of the plane normal (ensures to have an floor plane)
	double plane_max_height_; // maximum height of the detected plane above the mapped ground

	// multiscale search
	int detect_scales_; // Number of detection scales
	double bird_eye_base_resolution_; // base resolution of perspective transformation
	double bird_eye_start_resolution_; // smallest resolution for perspective transform
	cv::Mat multiscale_scores_; // scores for different scale detection
	cv::Size base_size_; // resolution of 2D projection under base resolution
	double image_scaling_;

	// further
	ros::Time last_incoming_message_;
	bool dirt_detection_callback_active_; ///< flag whether incoming messages shall be processed

public:

	/**
	 * Constructor.
	 */
	DirtDetection(ros::NodeHandle node_handle);

	/**
	 * Destructor.
	 */
	~DirtDetection();

	/**
	 * Create subscribers.
	 */
	void init();

	// dynamic reconfigure
	void dynamicReconfigureCallback(ipa_dirt_detection::DirtDetectionConfig &config, uint32_t level);

	/**
	 * Function is called if point cloud topic is received.
	 *
	 * @param [in] point_cloud2_rgb_msg	Point cloude message from camera.
	 *
	 */
	void dirtDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);

	void MultiScaleDirtDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);

	void planeLabelingCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);

	/**
	 * Converts: "sensor_msgs::Image::ConstPtr" \f$ \rightarrow \f$ "cv::Mat".
	 *	@param [in] 	color_image_msg 		Color image message from camera.
	 *	@param [in] 	color_image_ptr			See cv_bridge message to cv::Mat converter manual.
	 *	@param [out] 	color_image 			Color image from the message, in OpenCV representation.
	 */
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr,
			cv::Mat& color_image);

	/**
	 * Converts: "sensor_msgs::PointCloud2" \f$ \rightarrow \f$ "pcl::PointCloud<pcl::PointXYZRGB>::Ptr".
	 *	@param [in] 	point_cloud2_rgb_msg 		Point cloud message from camera.
	 *	@param [out] 	point_cloud_XYZRG 			Point cloud representation in PCL.
	 */
	void convertPointCloudMessageToPointCloudPcl(const sensor_msgs::PointCloud2::ConstPtr& point_cloud2_rgb_msg,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_XYZRGB);

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
	bool planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, const std_msgs::Header& header, cv::Mat& plane_color_image, cv::Mat& plane_mask,
			pcl::ModelCoefficients& plane_model, const tf::StampedTransform& transform_map_camera);

	/// remove perspective from image
	/// @param H Homography that maps points from the camera plane to the floor plane, i.e. pp = H*pc
	/// @param R Rotation matrix for transformation between floor plane and world coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
	/// @param t Translation vector. See Rotation matrix.
	/// @param cameraImagePlaneOffset Offset in the camera image plane. Conversion from floor plane to  [xc, yc]
	bool computeBirdsEyePerspective(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask,
			pcl::ModelCoefficients& plane_model, cv::Mat& H, cv::Mat& R, cv::Mat& t, cv::Point2f& cameraImagePlaneOffset, cv::Mat& plane_color_image_warped,
			cv::Mat& plane_mask_warped);

	//todo: dont know if this is needed anymore, cause of map is not used anymore(joel)
	/// converts point pointCamera, that lies within the floor plane and is provided in coordinates of the original camera image, into map coordinates
	void transformPointFromCameraImageToWorld(const cv::Mat& pointCamera, const cv::Mat& H, const cv::Mat& R, const cv::Mat& t,
			const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld);

	//todo: dont know if this is needed anymore, cause of map is not used anymore(joel)
	/// converts point pointPlane, that lies within the floor plane and is provided in coordinates of the warped camera image, into map coordinates
	void transformPointFromCameraWarpedToWorld(const cv::Mat& pointPlane, const cv::Mat& R, const cv::Mat& t, const cv::Point2f& cameraImagePlaneOffset,
			const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld);

	void transformPointFromWorldToCameraWarped(const cv::Point3f& pointWorld, const cv::Mat& R, const cv::Mat& t, const cv::Point2f& cameraImagePlaneOffset,
			const tf::StampedTransform& transformMapCamera, cv::Mat& pointPlane);

	//todo: dont know if this is needed anymore, cause of Image_Postprocessing_C1_rmb(joel)
	/**
	 * This function performs the saliency detection to spot dirt stains.
	 *
	 * @param [in] 	C1_image				!!!ONE CHANNEL!!!('C1') image used to perfom the salciency detection.
	 * @param [out]	C1_saliency_image		One channel image('C1') which results from the saliency detection.
	 */
	void SaliencyDetection_C1(const cv::Mat& C1_image, cv::Mat& C1_saliency_image);

	/**
	 * This function performs a saliency detection for a  3 channel color image.
	 *
	 * The function proceeds as follows:
	 * 						1.) The 3 channel image is split into their 3 channels.
	 * 						2.) The saliency detection is performed for each channel.
	 * 						3.) The resulting images are add up.
	 *
	 * @param [in] 	C3_color_image			!!!THREE CHANNEL!!!('C3') image used to perform the saliency detection.
	 * @param [in] 	mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 * @param [in]	gaussianBlurCycles		Determines the number of repetitions of the gaussian filter used to reduce the noise.
	 * @param [out]	C1_saliency_image		One channel image('C1') which results from the saliency detection.
	 */
	void SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask = 0, int gaussianBlurCycles = 2);

	/**
	 * This function uses the "C1_saliency_image" to mark the dirt in the "C3_color_image".
	 * Furthermore, it returns an image ("C1_BlackWhite_image") in which all dirt pixels are white (255) and all other pixels are black (0).
	 *
	 *
	 * @param [in]		C1_saliency_image		One channel('C1') saliency image used for postprocessing.
	 * @param [out] 	C1_BlackWhite_image		One channel('C1') image in which all dirt pixels are white (255) and all other pixels are black (0).
	 * @param [in,out]	C3_color_image			Three channel('C3') color which corresponds to the "C1_saliency_image".
	 */
	void Image_Postprocessing_C1(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image);

	/**
	 *
	 * This function avoids the false detections if no dirt is on the floor.
	 * Furthermore, it returns an image ("C1_BlackWhite_image") in which all dirt pixels are white (255) and all other pixels are black (0).
	 * It also the "C1_saliency_image" to mark the dirt in the "C3_color_image".
	 *
	 * @param [in] 		C1_saliency_image		One channel('C1') saliency image used for postprocessing.
	 * @param [in]		mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 * @param [in,out] 	C3_color_image			Three channel('C3') color which corresponds to the "C1_saliency_image".
	 * @param [out]		C1_BlackWhite_image		One channel('C1') image in which all dirt pixels are white (255) and all other pixels are black (0).
	 *
	 */
	void Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image,
			std::vector<cv::RotatedRect>& dirtDetections, const cv::Mat& mask = cv::Mat());

};
//end-class

}
;
//end-namespace

#endif /* DIRT_DETECTION_H_ */
