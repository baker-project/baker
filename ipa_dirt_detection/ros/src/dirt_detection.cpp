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

#include <ipa_dirt_detection/dirt_detection.h>
#include <ipa_dirt_detection/timer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <set>
#include <math.h>
#include <time.h>
#include <functional>

using namespace ipa_DirtDetection;

//#define WITH_MAP   // enables the usage of robot localization

struct lessPoint2i: public std::binary_function<cv::Point2i, cv::Point2i, bool>
{
	bool operator()(const cv::Point2i& a, const cv::Point2i& b) const
	{
		return ((a.x < b.x) || ((a.x == b.x) && (a.y < b.y)));
	}
};

/////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////

DirtDetection::DirtDetection(ros::NodeHandle node_handle) :
		node_handle_(node_handle), transform_listener_(node_handle)
{
	it_ = 0;
	last_incoming_message_ = ros::Time::now();
}

/////////////////////////////////////////////////
//  Destructor
/////////////////////////////////////////////////

DirtDetection::~DirtDetection()
{
	if (it_ != 0)
		delete it_;
}

/////////////////////////////////////////////////
// Create subscribers
/////////////////////////////////////////////////

void DirtDetection::init()
{
	// Parameters
	std::cout << "\n--------------------------\nDirt Detection Parameters:\n--------------------------\n";
	node_handle_.param("dirt_detection/spectralResidualGaussianBlurIterations", spectral_residual_gaussian_blur_iterations_, 2);
	std::cout << "spectralResidualGaussianBlurIterations = " << spectral_residual_gaussian_blur_iterations_ << std::endl;
	node_handle_.param("dirt_detection/dirtThreshold", dirt_threshold_, 0.5);
	std::cout << "dirtThreshold = " << dirt_threshold_ << std::endl;
	node_handle_.param("dirt_detection/spectralResidualNormalizationHighestMaxValue", spectral_residual_normalization_highest_max_value_, 1500.0);
	std::cout << "spectralResidualNormalizationHighestMaxValue = " << spectral_residual_normalization_highest_max_value_ << std::endl;
	node_handle_.param("dirt_detection/spectralResidualImageSizeRatio", spectral_residual_image_size_ratio_, 0.25);
	std::cout << "spectralResidualImageSizeRatio = " << spectral_residual_image_size_ratio_ << std::endl;
	node_handle_.param("dirt_detection/dirtCheckStdDevFactor", dirt_check_std_dev_factor_, 2.5);
	std::cout << "dirtCheckStdDevFactor = " << dirt_check_std_dev_factor_ << std::endl;
	node_handle_.param("dirt_detection/dirtDetectionActivatedOnStartup", dirt_detection_activated_on_startup_, true);
	std::cout << "dirtDetectionActivatedOnStartup = " << dirt_detection_activated_on_startup_ << std::endl;
	node_handle_.param("dirt_detection/warpImage", warp_image_, true);
	std::cout << "warpImage = " << warp_image_ << std::endl;
	node_handle_.param("dirt_detection/birdEyeResolution", bird_eye_resolution_, 300.0);
	std::cout << "birdEyeResolution = " << bird_eye_resolution_ << std::endl;
	node_handle_.param("dirt_detection/maxDistanceToCamera", max_distance_to_camera_, 300.0);
	std::cout << "maxDistanceToCamera = " << max_distance_to_camera_ << std::endl;
	node_handle_.param("dirt_detection/removeLines", remove_lines_, true);
	std::cout << "removeLines = " << remove_lines_ << std::endl;
	node_handle_.param("dirt_detection/floorSearchIterations", floor_search_iterations_, 3);
	std::cout << "floorSearchIterations = " << floor_search_iterations_ << std::endl;
	node_handle_.param("dirt_detection/minPlanePoints", min_plane_points_, 0);
	std::cout << "minPlanePoints = " << min_plane_points_ << std::endl;
	node_handle_.param("dirt_detection/planeNormalMaxZ", plane_normal_max_z_, -0.5);
	std::cout << "planeNormalMaxZ = " << plane_normal_max_z_ << std::endl;
	node_handle_.param("dirt_detection/planeMaxHeight", plane_max_height_, 0.3);
	std::cout << "planeMaxHeight = " << plane_max_height_ << std::endl;
	node_handle_.param("dirt_detection/dirtMappingMaskFilename", dirt_mapping_mask_filename_, std::string(""));
	std::cout << "dirtMappingMaskFilename = " << dirt_mapping_mask_filename_ << std::endl;
	node_handle_.param("dirt_detection/detectScales", detect_scales_, 5);
	std::cout << "detectScales = " << detect_scales_ << std::endl;
	node_handle_.param("dirt_detection/birdEyeStartResolution", bird_eye_start_resolution_, 75.0);
	std::cout << "birdEyeStartResolution = " << bird_eye_start_resolution_ << std::endl;
	node_handle_.param("dirt_detection/birdEyeBaseResolution", bird_eye_base_resolution_, 300.0);
	std::cout << "birdEyeBaseResolution = " << bird_eye_base_resolution_ << std::endl;

	node_handle_.param("dirt_detection/showOriginalImage", debug_["showOriginalImage"], false);
	std::cout << "showOriginalImage = " << debug_["showOriginalImage"] << std::endl;
	node_handle_.param("dirt_detection/showPlaneColorImage", debug_["showPlaneColorImage"], false);
	std::cout << "showPlaneColorImage = " << debug_["showPlaneColorImage"] << std::endl;
	node_handle_.param("dirt_detection/showWarpedOriginalImage", debug_["showWarpedOriginalImage"], false);
	std::cout << "showWarpedOriginalImage = " << debug_["showWarpedOriginalImage"] << std::endl;
	node_handle_.param("dirt_detection/showSaliencyBadScale", debug_["showSaliencyBadScale"], false);
	std::cout << "showSaliencyBadScale = " << debug_["showSaliencyBadScale"] << std::endl;
	node_handle_.param("dirt_detection/showColorWithArtificialDirt", debug_["showColorWithArtificialDirt"], false);
	std::cout << "showColorWithArtificialDirt = " << debug_["showColorWithArtificialDirt"] << std::endl;
	node_handle_.param("dirt_detection/showSaliencyWithArtificialDirt", debug_["showSaliencyWithArtificialDirt"], false);
	std::cout << "showSaliencyWithArtificialDirt = " << debug_["showSaliencyWithArtificialDirt"] << std::endl;
	node_handle_.param("dirt_detection/showSaliencyDetection", debug_["showSaliencyDetection"], false);
	std::cout << "showSaliencyDetection = " << debug_["showSaliencyDetection"] << std::endl;
	node_handle_.param("dirt_detection/showDetectedLines", debug_["showDetectedLines"], false);
	std::cout << "showDetectedLines = " << debug_["showDetectedLines"] << std::endl;
	node_handle_.param("dirt_detection/showDirtDetections", debug_["showDirtDetections"], false);
	std::cout << "showDirtDetections = " << debug_["showDirtDetections"] << std::endl;
	node_handle_.param("dirt_detection/publishDirtDetections", debug_["publishDirtDetections"], false);
	std::cout << "publishDirtDetections = " << debug_["publishDirtDetections"] << std::endl;
	node_handle_.param("dirt_detection/showObservationsGrid", debug_["showObservationsGrid"], false);
	std::cout << "showObservationsGrid = " << debug_["showObservationsGrid"] << std::endl;
	node_handle_.param("dirt_detection/showDirtGrid", debug_["showDirtGrid"], false);
	std::cout << "showDirtGrid = " << debug_["showDirtGrid"] << std::endl;
	node_handle_.param("dirt_detection/SaveDataForTest", debug_["SaveDataForTest"], false);
	std::cout << "SaveDataForTest = " << debug_["SaveDataForTest"] << std::endl;
	node_handle_.param("dirt_detection/MultiScaleDebug", debug_["MultiScaleDebug"], false);
	std::cout << "MultiScaleDebug = " << debug_["MultiScaleDebug"] << std::endl;

	// dynamic reconfigure
	dynamic_reconfigure::Server<ipa_dirt_detection::DirtDetectionConfig>::CallbackType dynamic_reconfigure_callback_type;
	dynamic_reconfigure_callback_type = boost::bind(&DirtDetection::dynamicReconfigureCallback, this, _1, _2);
	dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_type);

	it_ = new image_transport::ImageTransport(node_handle_);
//	color_camera_image_sub_ = it_->subscribe("image_color", 1, boost::bind(&DirtDetection::imageDisplayCallback, this, _1));
	dirt_detection_image_pub_ = it_->advertise("dirt_detections", 1);

	// dirt detection on at the beginning?
	dirt_detection_callback_active_ = dirt_detection_activated_on_startup_;

	camera_depth_points_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 1, &DirtDetection::dirtDetectionCallback, this);

	// services
	activate_dirt_detection_service_server_ = node_handle_.advertiseService("activate_dirt_detection", &DirtDetection::activateDirtDetection, this);
	deactivate_dirt_detection_service_server_ = node_handle_.advertiseService("deactivate_dirt_detection", &DirtDetection::deactivateDirtDetection, this);

	floor_plane_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("floor_plane", 1);
}

void DirtDetection::dynamicReconfigureCallback(ipa_dirt_detection::DirtDetectionConfig &config, uint32_t level)
{
	//ROS_INFO("Reconfigure Request: %d %f %s %s %d",	config.int_param, config.double_param, config.str_param.c_str(), config.bool_param?"True":"False", config.size);
	dirt_threshold_ = config.dirtThreshold;
	warp_image_ = config.warpImage;
	bird_eye_resolution_ = config.birdEyeResolution;
	max_distance_to_camera_ = config.maxDistanceToCamera;
	remove_lines_ = config.removeLines;
	floor_search_iterations_ = config.floorSearchIterations;
	min_plane_points_ = config.minPlanePoints;
	std::cout << "Dynamic reconfigure changed settings to \n";
	std::cout << "  dirtThreshold = " << dirt_threshold_ << std::endl;
	std::cout << "  warpImage = " << warp_image_ << std::endl;
	std::cout << "  birdEyeResolution = " << bird_eye_resolution_ << std::endl;
	std::cout << "  maxDistanceToCamera = " << max_distance_to_camera_ << std::endl;
	std::cout << "  removeLines = " << remove_lines_ << std::endl;
	std::cout << "  floorSearchIterations = " << floor_search_iterations_ << std::endl;
	std::cout << "  minPlanePoints = " << min_plane_points_ << std::endl;
}

bool DirtDetection::activateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ROS_INFO("Activating dirt detection.");
	dirt_detection_callback_active_ = true;

	res.success = true;

	return true;
}

bool DirtDetection::deactivateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ROS_INFO("Deactivating dirt detection.");
	dirt_detection_callback_active_ = false;

	res.success = true;

	return true;
}

void DirtDetection::dirtDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{
	if (dirt_detection_callback_active_ == false)
		return;

	// get tf between camera and map
	tf::StampedTransform transformMapCamera;
	transformMapCamera.setIdentity();

	// convert point cloud message
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	convertPointCloudMessageToPointCloudPcl(point_cloud2_rgb_msg, input_cloud);
	std::cout << input_cloud->size();

	Timer tim;
	double segmentation_time = 0., dirt_detection_time = 0.;

	// find ground plane
	cv::Mat plane_color_image = cv::Mat();
	cv::Mat plane_mask = cv::Mat();
	pcl::ModelCoefficients plane_model;
	bool found_plane = planeSegmentation(input_cloud, point_cloud2_rgb_msg->header, plane_color_image, plane_mask, plane_model, transformMapCamera);

	// todo: paramter for mask usage or not
	// if not:
	//plane_mask.setTo(cv::Scalar(255));	// all points belong to plane

	//std::cout << "Segmentation time: " << tim.getElapsedTimeInMilliSec() << "ms." << std::endl;
	segmentation_time = tim.getElapsedTimeInMilliSec();
	tim.start();

	// check if a ground plane could be found
	if (found_plane == true)
	{
		//cv::cvtColor(plane_color_image, plane_color_image, CV_BGR2Lab);

//		cv::Mat laplace;
//		cv::Laplacian(plane_color_image, laplace, CV_32F, 5);
//		laplace = laplace.mul(laplace);
//		cv::normalize(laplace, laplace, 0, 1, NORM_MINMAX);
//		cv:imshow("laplace", laplace);

// test with half-scale image
//		cv::Mat temp = plane_color_image;
//		cv::resize(temp, plane_color_image, cv::Size(), 0.5, 0.5);
//		temp = plane_mask;
//		cv::resize(temp, plane_mask, cv::Size(), 0.5, 0.5);

		// remove perspective from image
		cv::Mat H; // homography between floor plane in image and bird's eye perspective
		cv::Mat R, t; // transformation between world and floor plane coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
		cv::Point2f camera_image_plane_offset; // offset in the camera image plane. Conversion from floor plane to  [xc, yc]
		cv::Mat plane_color_image_warped;
		cv::Mat plane_mask_warped;
		if (warp_image_ == true)
		{
			bool transformSuccessful = computeBirdsEyePerspective(input_cloud, plane_color_image, plane_mask, plane_model, H, R, t, camera_image_plane_offset,
					plane_color_image_warped, plane_mask_warped);
			if (transformSuccessful == false)
				return;
		}
		else
		{
			H = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
			R = H;
			t = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
			camera_image_plane_offset.x = 0.f;
			camera_image_plane_offset.y = 0.f;
			plane_color_image_warped = plane_color_image;
			plane_mask_warped = plane_mask;
		}

		// detect dirt on the floor
		cv::Mat C1_saliency_image;
		SaliencyDetection_C3(plane_color_image_warped, C1_saliency_image, &plane_mask_warped, spectral_residual_gaussian_blur_iterations_);

		// post processing, dirt/stain selection
		cv::Mat C1_BlackWhite_image;
		cv::Mat new_plane_color_image = plane_color_image_warped.clone();
		std::vector<cv::RotatedRect> dirtDetections;
		Image_Postprocessing_C1_rmb(C1_saliency_image, C1_BlackWhite_image, new_plane_color_image, dirtDetections, plane_mask_warped);

#ifdef WITH_MAP
		// convert detections to map coordinates and mark dirt regions in map
		for (int i=0; i<(int)dirtDetections.size(); i++)
		{
			labelImage::RegionPointTriple pointsWorldMap;

			// center point
			cv::Mat pc;
			if (warp_image_ == true)
			pc = (cv::Mat_<double>(3,1) << (double)dirtDetections[i].center.x, (double)dirtDetections[i].center.y, 1.0);
			else
			pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			transformPointFromCameraWarpedToWorld(pc, R, t, camera_image_plane_offset, transformMapCamera, pointsWorldMap.center);
//			cv::Mat pc_copy;
//			transformPointFromWorldToCameraWarped(pointsWorldMap.center, R, t, cameraImagePlaneOffset, transformMapCamera, pc_copy);
//			std::cout << " pc=(" << pc.at<double>(0) << ", " << pc.at<double>(1) << ", " << pc.at<double>(2) << ")    pc_copy=(" << pc_copy.at<double>(0) << ", " << pc_copy.at<double>(1) << ", " << pc_copy.at<double>(2) << ")\n";
//			std::cout << "---------- world.x=" << pointsWorldMap.center.x << "   world.y=" << pointsWorldMap.center.y << "   world.z=" << pointsWorldMap.center.z << std::endl;

			// point in width direction
			double u = (double)dirtDetections[i].center.x+cos(-dirtDetections[i].angle*3.14159265359/180.f)*dirtDetections[i].size.width/2.f;//todo: offset?
			double v = (double)dirtDetections[i].center.y-sin(-dirtDetections[i].angle*3.14159265359/180.f)*dirtDetections[i].size.width/2.f;
			//std::cout << "dd: " << dirtDetections[i].center.x << " " << dirtDetections[i].center.y << "  u:" << u << "  v:" << v;
			if (warp_image_ == true)
			pc = (cv::Mat_<double>(3,1) << u, v, 1.0);
			else
			//pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].x, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].y, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].z);
			pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			transformPointFromCameraWarpedToWorld(pc, R, t, camera_image_plane_offset, transformMapCamera, pointsWorldMap.p1);

			// point in height direction
			u = (double)dirtDetections[i].center.x-cos((-dirtDetections[i].angle-90)*3.14159265359/180.f)*dirtDetections[i].size.height/2.f;
			v = (double)dirtDetections[i].center.y-sin((-dirtDetections[i].angle-90)*3.14159265359/180.f)*dirtDetections[i].size.height/2.f;
			//std::cout << "   uh:" << u << "   vh:" << v << std::endl;
			if (warp_image_ == true)
			pc = (cv::Mat_<double>(3,1) << u, v, 1.0);
			else
			//pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].x, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].y, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].z);
			pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			transformPointFromCameraWarpedToWorld(pc, R, t, camera_image_plane_offset, transformMapCamera, pointsWorldMap.p2);

			putDetectionIntoGrid(gridPositiveVotes_, pointsWorldMap);
		}

		if (debug_["showDirtGrid"] == true)
		{
			cv::Mat gridPositiveVotesDisplay;
			cv::normalize(gridPositiveVotes_, gridPositiveVotesDisplay, 0., 255*256., cv::NORM_MINMAX);
			cv::imshow("dirt grid", gridPositiveVotesDisplay);
			cvMoveWindow("dirt grid", 0, 0);
		}

		// todo: new mode with dirt deletion:
//		cv::Point2i offset(0,0);//gridNumberObservations_.cols/2, gridNumberObservations_.rows/2);		//done: offset
		cv::Mat Rt = R.t();
		cv::Mat Rtt = Rt*t;
		for (int v=0; v<gridNumberObservations_.rows; v++)
		{
			for (int u=0; u<gridNumberObservations_.cols; u++)
			{
				// only update currently visible cells
				if (gridNumberObservations_.at<int>(v,u) != 0)
				{
					if (warp_image_ == true)
					{
						// todo: compute more efficiently
						// only mark grid cells as observed if they are part of the warped image
						//tf::Vector3 pointWorldMapBt(-(u-offset.x)/gridResolution_ + gridOrigin_.x, (v-offset.y)/gridResolution_ + gridOrigin_.y, 0.0);		//done: offset
						tf::Vector3 pointWorldMapBt(u/gridResolution_ + gridOrigin_.x, v/gridResolution_ + gridOrigin_.y, 0.0);
						tf::Vector3 pointWorldCameraBt = transformMapCamera.inverse() * pointWorldMapBt;// transform map point (world coordinates) into camera coordinates
						cv::Mat pointWorldCamera = (cv::Mat_<double>(3,1) << pointWorldCameraBt.getX(), pointWorldCameraBt.getY(), pointWorldCameraBt.getZ());
						cv::Mat pointFloorPlane = Rt*pointWorldCamera - Rtt;// =point in detected floor plane in plane coordinate system
						cv::Mat pointPlaneImage = (cv::Mat_<double>(3,1) << (pointFloorPlane.at<double>(0)-camera_image_plane_offset.x)*bird_eye_resolution_, (pointFloorPlane.at<double>(1)-camera_image_plane_offset.y)*bird_eye_resolution_, 1.0);
						// todo: parameter candidate?
						double borderOffset = 30.;// pixel distance from image border - observations close to the border should not count as there are no detections happening
						if (pointPlaneImage.at<double>(0) < 0.+borderOffset || pointPlaneImage.at<double>(0) > new_plane_color_image.cols-borderOffset ||
								pointPlaneImage.at<double>(1) < 0.+borderOffset || pointPlaneImage.at<double>(1) > new_plane_color_image.rows-borderOffset)
						{
							gridNumberObservations_.at<int>(v,u) = 0;
							continue;
						}
					}

					// update history of cell values
					historyLastEntryIndex_.at<int>(v,u) = (historyLastEntryIndex_.at<int>(v,u)+1)%detectionHistoryDepth_;
					listOfLastDetections_[u][v][historyLastEntryIndex_.at<int>(v,u)] = (gridPositiveVotes_.at<int>(v,u)!=0 ? 1 : 0);
				}
			}
		}

		// create occupancy grid map from detections
		nav_msgs::OccupancyGrid detectionMap;
		createOccupancyGridMapFromDirtDetections(detectionMap);
		detection_map_pub_.publish(detectionMap);

		//std::cout << "Dirt Detection time: " << tim.getElapsedTimeInMilliSec() << "ms." << std::endl;
		dirt_detection_time = tim.getElapsedTimeInMilliSec();
		meanProcessingTimeSegmentation_ = (meanProcessingTimeSegmentation_*rosbagMessagesProcessed_+segmentation_time)/(rosbagMessagesProcessed_+1.0);
		meanProcessingTimeDirtDetection_ = (meanProcessingTimeDirtDetection_*rosbagMessagesProcessed_+dirt_detection_time)/(rosbagMessagesProcessed_+1.0);
		std::cout << "mean times for segmentation, dirt detection, total:\t" << meanProcessingTimeSegmentation_ << "\t" << meanProcessingTimeDirtDetection_ << "\t" << meanProcessingTimeSegmentation_+meanProcessingTimeDirtDetection_ << std::endl;

		// store data internally if necessary
		if (storeLastImage_ == true)
		{
			boost::mutex::scoped_lock lock(storeLastImageMutex_);

			lastImageDataStorage_.plane_color_image_warped = plane_color_image_warped;
			lastImageDataStorage_.R = R;
			lastImageDataStorage_.t = t;
			lastImageDataStorage_.camera_image_plane_offset = camera_image_plane_offset;
			lastImageDataStorage_.transformMapCamera = transformMapCamera;
		}

		// publish image
		if (debug_["publishDirtDetections"] == true)
		{
			cv_bridge::CvImage cv_ptr;
			cv_ptr.image = new_plane_color_image;
			cv_ptr.encoding = "bgr8";
			dirt_detection_image_pub_.publish(cv_ptr.toImageMsg());
		}
//
//		if (debug_["showObservationsGrid"] == true)
//		{
//			cv::Mat gridObservationsDisplay;
//			cv::normalize(gridNumberObservations_, gridObservationsDisplay, 0., 255*256., cv::NORM_MINMAX);
//			cv::imshow("observations grid", gridObservationsDisplay);
//			cvMoveWindow("observations grid", 340, 0);
//		}

		// publish image of map and dirt spots
//		cv::Mat map_with_dirt(gridPositiveVotes_.rows, gridPositiveVotes_.cols, CV_8UC3);
//		map_with_dirt.setTo(cv::Scalar(255,255,255));
//		double scale = 1./(floor_plan_.info.resolution*gridResolution_);
//		for (int v=0, i=0; v<gridPositiveVotes_.rows; v++)
//		{
//			for (int u=0; u<gridPositiveVotes_.cols; u++, i++)
//			{
//				int index = v*scale*gridPositiveVotes_.cols*scale + u*scale;
//				map_with_dirt.at<cv::Vec3b>(v,gridPositiveVotes_.cols-u) = cv::Vec3b((100-floor_plan_.data[index])*2.55,(100-floor_plan_.data[index])*2.55,(100-floor_plan_.data[index])*2.55);
//				if (detectionMap.data[i] == 100)
//					map_with_dirt.at<cv::Vec3b>(v,gridPositiveVotes_.cols-u) = cv::Vec3b(0,0,255);
//			}
//		}
//		cv_ptr.image = map_with_dirt;
//		cv_ptr.encoding = "bgr8";
//		dirt_detection_image_with_map_pub_.publish(cv_ptr.toImageMsg());
#endif

		if (debug_["showWarpedOriginalImage"] == true)
		{
			cv::imshow("warped original image", plane_color_image_warped);
			//cvMoveWindow("dirt grid", 0, 0);
			cv::waitKey(10);
		}

		if (debug_["showDirtDetections"] == true)
		{
			cv::imshow("dirt detections", new_plane_color_image);
			cvMoveWindow("dirt detections", 650, 530);
			cv::waitKey(10);
		}

		if (debug_["showPlaneColorImage"] == true)
		{
			cv::imshow("segmented color image", plane_color_image);
			cvMoveWindow("segmented color image", 650, 0);
			cv::waitKey(10);
		}

		if (debug_["SaveDataForTest"] == true)
		{
//			std::stringstream ss2;
//			ss2 << frame_num_bag;
//			framenumbag = ss2.str();
//			std::cout << "current frame is num: " << framenumbag << std::endl;
//			cv::imwrite("test_data/dir_dect_" + birdeyeresolution + "_" + framenumbag + ".jpg", new_plane_color_image);
		}
	}
}

void DirtDetection::MultiScaleDirtDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{
	if (dirt_detection_callback_active_ == false)
		return;

	// get tf between camera and map
	tf::StampedTransform transform_map_camera;
	transform_map_camera.setIdentity();

	base_size_ = cv::Size(640, 480);
	multiscale_scores_ = cv::Mat::zeros(base_size_, CV_32F);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	convertPointCloudMessageToPointCloudPcl(point_cloud2_rgb_msg, input_cloud);

	Timer tim;
	double segmentation_time = 0., dirt_detection_time = 0.;

	// find ground plane
	cv::Mat plane_color_image = cv::Mat();
	cv::Mat plane_mask = cv::Mat();
	pcl::ModelCoefficients plane_model;
	bool found_plane = planeSegmentation(input_cloud, point_cloud2_rgb_msg->header, plane_color_image, plane_mask, plane_model, transform_map_camera);

	//std::cout << "Segmentation time: " << tim.getElapsedTimeInMilliSec() << "ms." << std::endl;
	segmentation_time = tim.getElapsedTimeInMilliSec();
	tim.start();

	// check if a ground plane could be found
	if (found_plane == true)
	{
		//cv::cvtColor(plane_color_image, plane_color_image, CV_BGR2Lab);

//		cv::Mat laplace;
//		cv::Laplacian(plane_color_image, laplace, CV_32F, 5);
//		laplace = laplace.mul(laplace);
//		cv::normalize(laplace, laplace, 0, 1, NORM_MINMAX);
//		cv:imshow("laplace", laplace);

		// test with half-scale image
//		cv::Mat temp = plane_color_image;
//		cv::resize(temp, plane_color_image, cv::Size(), 0.5, 0.5);
//		temp = plane_mask;
//		cv::resize(temp, plane_mask, cv::Size(), 0.5, 0.5);

		// remove perspective from image
		for (int s = 0; s < detect_scales_; s++)
		{
			cv::Mat H; // homography between floor plane in image and bird's eye perspective
			cv::Mat R, t; // transformation between world and floor plane coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
			cv::Point2f camera_image_plane_offset; // offset in the camera image plane. Conversion from floor plane to  [xc, yc]
			cv::Mat plane_color_image_warped;
			cv::Mat plane_mask_warped;
			image_scaling_ = pow(2, s) * bird_eye_start_resolution_ / bird_eye_base_resolution_; // todo: make the standard resolution of 300 a parameter
			bird_eye_resolution_ = pow(2, s) * bird_eye_start_resolution_;

			if (warp_image_ == true)
			{
				bool transformSuccessful = computeBirdsEyePerspective(input_cloud, plane_color_image, plane_mask, plane_model, H, R, t, camera_image_plane_offset,
						plane_color_image_warped, plane_mask_warped);
				if (transformSuccessful == false)
					return;
			}
			else
			{
				H = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
				R = H;
				t = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
				camera_image_plane_offset.x = 0.f;
				camera_image_plane_offset.y = 0.f;
				plane_color_image_warped = plane_color_image;
				plane_mask_warped = plane_mask;
			}

			// detect dirt on the floor
			cv::Mat C1_saliency_image;
			SaliencyDetection_C3(plane_color_image_warped, C1_saliency_image, &plane_mask_warped, spectral_residual_gaussian_blur_iterations_);

			// post processing, dirt/stain selection
			cv::Mat C1_BlackWhite_image;
			cv::Mat new_plane_color_image = plane_color_image_warped.clone();
			std::vector<cv::RotatedRect> dirtDetections;

			Image_Postprocessing_C1_rmb(C1_saliency_image, C1_BlackWhite_image, new_plane_color_image, dirtDetections, plane_mask_warped);
			/*
			 #ifdef WITH_MAP
			 // convert detections to map coordinates and mark dirt regions in map
			 for (int i=0; i<(int)dirtDetections.size(); i++)
			 {
			 labelImage::RegionPointTriple pointsWorldMap;

			 // center point
			 cv::Mat pc;
			 if (warpImage_ == true)
			 pc = (cv::Mat_<double>(3,1) << (double)dirtDetections[i].center.x, (double)dirtDetections[i].center.y, 1.0);
			 else
			 pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			 transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.center);
			 //			cv::Mat pc_copy;
			 //			transformPointFromWorldToCameraWarped(pointsWorldMap.center, R, t, cameraImagePlaneOffset, transformMapCamera, pc_copy);
			 //			std::cout << " pc=(" << pc.at<double>(0) << ", " << pc.at<double>(1) << ", " << pc.at<double>(2) << ")    pc_copy=(" << pc_copy.at<double>(0) << ", " << pc_copy.at<double>(1) << ", " << pc_copy.at<double>(2) << ")\n";
			 //			std::cout << "---------- world.x=" << pointsWorldMap.center.x << "   world.y=" << pointsWorldMap.center.y << "   world.z=" << pointsWorldMap.center.z << std::endl;

			 // point in width direction
			 double u = (double)dirtDetections[i].center.x+cos(-dirtDetections[i].angle*3.14159265359/180.f)*dirtDetections[i].size.width/2.f;	//todo: offset?
			 double v = (double)dirtDetections[i].center.y-sin(-dirtDetections[i].angle*3.14159265359/180.f)*dirtDetections[i].size.width/2.f;
			 //std::cout << "dd: " << dirtDetections[i].center.x << " " << dirtDetections[i].center.y << "  u:" << u << "  v:" << v;
			 if (warpImage_ == true)
			 pc = (cv::Mat_<double>(3,1) << u, v, 1.0);
			 else
			 //pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].x, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].y, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].z);
			 pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			 transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.p1);

			 // point in height direction
			 u = (double)dirtDetections[i].center.x-cos((-dirtDetections[i].angle-90)*3.14159265359/180.f)*dirtDetections[i].size.height/2.f;
			 v = (double)dirtDetections[i].center.y-sin((-dirtDetections[i].angle-90)*3.14159265359/180.f)*dirtDetections[i].size.height/2.f;
			 //std::cout << "   uh:" << u << "   vh:" << v << std::endl;
			 if (warpImage_ == true)
			 pc = (cv::Mat_<double>(3,1) << u, v, 1.0);
			 else
			 //pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].x, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].y, (double)(*input_cloud)[(int)v*input_cloud->width+(int)u].z);
			 pc = (cv::Mat_<double>(3,1) << (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].x, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].y, (double)(*input_cloud)[dirtDetections[i].center.y*input_cloud->width+dirtDetections[i].center.x].z);
			 transformPointFromCameraWarpedToWorld(pc, R, t, cameraImagePlaneOffset, transformMapCamera, pointsWorldMap.p2);

			 putDetectionIntoGrid(gridPositiveVotes_, pointsWorldMap);
			 }

			 if (debug_["showDirtGrid"] == true)
			 {
			 cv::Mat gridPositiveVotesDisplay;
			 cv::normalize(gridPositiveVotes_, gridPositiveVotesDisplay, 0., 255*256., cv::NORM_MINMAX);
			 cv::imshow("dirt grid", gridPositiveVotesDisplay);
			 cvMoveWindow("dirt grid", 0, 0);
			 }

			 // todo: new mode with dirt deletion:
			 //		cv::Point2i offset(0,0);//gridNumberObservations_.cols/2, gridNumberObservations_.rows/2);		//done: offset
			 cv::Mat Rt = R.t();
			 cv::Mat Rtt = Rt*t;
			 for (int v=0; v<gridNumberObservations_.rows; v++)
			 {
			 for (int u=0; u<gridNumberObservations_.cols; u++)
			 {
			 // only update currently visible cells
			 if (gridNumberObservations_.at<int>(v,u) != 0)
			 {
			 if (warpImage_ == true)
			 {
			 // todo: compute more efficiently
			 // only mark grid cells as observed if they are part of the warped image
			 //tf::Vector3 pointWorldMapBt(-(u-offset.x)/gridResolution_ + gridOrigin_.x, (v-offset.y)/gridResolution_ + gridOrigin_.y, 0.0);		//done: offset
			 tf::Vector3 pointWorldMapBt(u/gridResolution_ + gridOrigin_.x, v/gridResolution_ + gridOrigin_.y, 0.0);
			 tf::Vector3 pointWorldCameraBt = transformMapCamera.inverse() * pointWorldMapBt;	// transform map point (world coordinates) into camera coordinates
			 cv::Mat pointWorldCamera = (cv::Mat_<double>(3,1) << pointWorldCameraBt.getX(), pointWorldCameraBt.getY(), pointWorldCameraBt.getZ());
			 cv::Mat pointFloorPlane = Rt*pointWorldCamera - Rtt; 	// =point in detected floor plane in plane coordinate system
			 cv::Mat pointPlaneImage = (cv::Mat_<double>(3,1) << (pointFloorPlane.at<double>(0)-cameraImagePlaneOffset.x)*birdEyeResolution_, (pointFloorPlane.at<double>(1)-cameraImagePlaneOffset.y)*birdEyeResolution_, 1.0);
			 // todo: parameter candidate?
			 double borderOffset = 30.;	// pixel distance from image border - observations close to the border should not count as there are no detections happening
			 if (pointPlaneImage.at<double>(0) < 0.+borderOffset || pointPlaneImage.at<double>(0) > new_plane_color_image.cols-borderOffset ||
			 pointPlaneImage.at<double>(1) < 0.+borderOffset || pointPlaneImage.at<double>(1) > new_plane_color_image.rows-borderOffset)
			 {
			 gridNumberObservations_.at<int>(v,u) = 0;
			 continue;
			 }
			 }

			 // update history of cell values
			 historyLastEntryIndex_.at<int>(v,u) = (historyLastEntryIndex_.at<int>(v,u)+1)%detectionHistoryDepth_;
			 listOfLastDetections_[u][v][historyLastEntryIndex_.at<int>(v,u)] = (gridPositiveVotes_.at<int>(v,u)!=0 ? 1 : 0);
			 }
			 }
			 }

			 // create occupancy grid map from detections
			 nav_msgs::OccupancyGrid detectionMap;
			 createOccupancyGridMapFromDirtDetections(detectionMap);
			 detection_map_pub_.publish(detectionMap);

			 //std::cout << "Dirt Detection time: " << tim.getElapsedTimeInMilliSec() << "ms." << std::endl;
			 dirtDetectionTime = tim.getElapsedTimeInMilliSec();
			 meanProcessingTimeSegmentation_ = (meanProcessingTimeSegmentation_*rosbagMessagesProcessed_+segmentationTime)/(rosbagMessagesProcessed_+1.0);
			 meanProcessingTimeDirtDetection_ = (meanProcessingTimeDirtDetection_*rosbagMessagesProcessed_+dirtDetectionTime)/(rosbagMessagesProcessed_+1.0);
			 std::cout << "mean times for segmentation, dirt detection, total:\t" << meanProcessingTimeSegmentation_ << "\t" << meanProcessingTimeDirtDetection_ << "\t" << meanProcessingTimeSegmentation_+meanProcessingTimeDirtDetection_ << std::endl;

			 // store data internally if necessary
			 if (storeLastImage_ == true)
			 {
			 boost::mutex::scoped_lock lock(storeLastImageMutex_);

			 lastImageDataStorage_.plane_color_image_warped = plane_color_image_warped;
			 lastImageDataStorage_.R = R;
			 lastImageDataStorage_.t = t;
			 lastImageDataStorage_.cameraImagePlaneOffset = cameraImagePlaneOffset;
			 lastImageDataStorage_.transformMapCamera = transformMapCamera;
			 }

			 // publish image
			 if (debug_["publishDirtDetections"] == true)
			 {
			 cv_bridge::CvImage cv_ptr;
			 cv_ptr.image = new_plane_color_image;
			 cv_ptr.encoding = "bgr8";
			 dirt_detection_image_pub_.publish(cv_ptr.toImageMsg());
			 }
			 //
			 //		if (debug_["showObservationsGrid"] == true)
			 //		{
			 //			cv::Mat gridObservationsDisplay;
			 //			cv::normalize(gridNumberObservations_, gridObservationsDisplay, 0., 255*256., cv::NORM_MINMAX);
			 //			cv::imshow("observations grid", gridObservationsDisplay);
			 //			cvMoveWindow("observations grid", 340, 0);
			 //		}

			 // publish image of map and dirt spots
			 //		cv::Mat map_with_dirt(gridPositiveVotes_.rows, gridPositiveVotes_.cols, CV_8UC3);
			 //		map_with_dirt.setTo(cv::Scalar(255,255,255));
			 //		double scale = 1./(floor_plan_.info.resolution*gridResolution_);
			 //		for (int v=0, i=0; v<gridPositiveVotes_.rows; v++)
			 //		{
			 //			for (int u=0; u<gridPositiveVotes_.cols; u++, i++)
			 //			{
			 //				int index = v*scale*gridPositiveVotes_.cols*scale + u*scale;
			 //				map_with_dirt.at<cv::Vec3b>(v,gridPositiveVotes_.cols-u) = cv::Vec3b((100-floor_plan_.data[index])*2.55,(100-floor_plan_.data[index])*2.55,(100-floor_plan_.data[index])*2.55);
			 //				if (detectionMap.data[i] == 100)
			 //					map_with_dirt.at<cv::Vec3b>(v,gridPositiveVotes_.cols-u) = cv::Vec3b(0,0,255);
			 //			}
			 //		}
			 //		cv_ptr.image = map_with_dirt;
			 //		cv_ptr.encoding = "bgr8";
			 //		dirt_detection_image_with_map_pub_.publish(cv_ptr.toImageMsg());
			 #endif
			 */

			if (debug_["showWarpedOriginalImage"] == true)
			{
				cv::imshow("warped original image", plane_color_image_warped);
				//cvMoveWindow("dirt grid", 0, 0);
				cv::waitKey(10);
			}

			if (debug_["showDirtDetections"] == true)
			{
				cv::imshow("dirt detections", new_plane_color_image);
				cvMoveWindow("dirt detections", 650, 530);
				cv::waitKey(10);
			}

			if (debug_["showPlaneColorImage"] == true)
			{
				cv::imshow("segmented color image", plane_color_image);
				cvMoveWindow("segmented color image", 650, 0);
				cv::waitKey(10);
			}

//			if (debug_["SaveDataForTest"] == true)
//			{
//				std::stringstream ss2;
//				ss2 << frame_num_bag;
//				framenumbag = ss2.str();
//				std::cout << "current frame is num: " << framenumbag << std::endl;
//				cv::imwrite("test_data/dir_dect_" + birdeyeresolution + "_" + framenumbag + ".jpg", new_plane_color_image);
//			}
		}
	}
}

/////////////////////////////////////////////////
// Convert functions
/////////////////////////////////////////////////

void DirtDetection::convertPointCloudMessageToPointCloudPcl(const sensor_msgs::PointCloud2::ConstPtr& point_cloud2_rgb_msg,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_XYZRGB)
{
	//conversion Ros message->Pcl point cloud
	pcl::fromROSMsg(*point_cloud2_rgb_msg, *point_cloud_XYZRGB);
}

unsigned long DirtDetection::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr,
		cv::Mat& color_image)
{
	try
	{
		color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("DirtDetection: cv_bridge exception: %s", e.what());
		return 1;
	}
	color_image = color_image_ptr->image;

	return 0;
}

bool DirtDetection::planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, const std_msgs::Header& header, cv::Mat& plane_color_image,
		cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, const tf::StampedTransform& transform_map_camera)
{

	//recreate original color image from point cloud
	if (debug_["showOriginalImage"] == true)
	{
		cv::Mat color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
		int index = 0;
		for (int v = 0; v < (int) input_cloud->height; v++)
		{
			for (int u = 0; u < (int) input_cloud->width; u++, index++)
			{
				pcl::PointXYZRGB point = (*input_cloud)[index];
				color_image.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, point.r);
			}
		}
		//display original image
		cv::imshow("original color image", color_image);
		cvMoveWindow("original color image", 0, 0);
		cv::waitKey(50);
		if (debug_["SaveDataForTest"] == true)
		{
			cv::imwrite("test_data/ori_image" + birdeyeresolution + ".jpg", color_image);
		}
		//cvMoveWindow("color image", 0, 520);
	}

	// try several times to find the ground plane
	double plane_inlier_threshold = 0.05; // cm
	bool found_plane = false;

	// downsample the dataset with a voxel filter using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud(input_cloud->makeShared());
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*filtered_input_cloud);
	//std::cout << "PointCloud after filtering has: " << filtered_input_cloud->points.size ()  << " data points." << std::endl;

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	for (int trial = 0; (trial < floor_search_iterations_ && filtered_input_cloud->points.size() > 100); trial++)
	{
		// Create the segmentation object for the planar model and set all the parameters
		inliers->indices.clear();
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(plane_inlier_threshold);
		seg.setInputCloud(filtered_input_cloud);
		seg.segment(*inliers, plane_model);

		// keep plane_normal upright
		if (plane_model.values[2] < 0.)
		{
			plane_model.values[0] *= -1;
			plane_model.values[1] *= -1;
			plane_model.values[2] *= -1;
			plane_model.values[3] *= -1;
		}

		// verify that plane is a valid ground plane
		if (inliers->indices.size() != 0)//			tf::Vector3 planePointCamera(point.x, point.y, point.z);
			//			tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
			//			//cv::Point2i co(-(planePointWorld.getX()-gridOrigin_.x)*gridResolution_+grid_offset.x, (planePointWorld.getY()-gridOrigin_.y)*gridResolution_+grid_offset.y);	//done: offset
			//			cv::Point2i co((planePointWorld.getX() - gridOrigin_.x) * gridResolution_, (planePointWorld.getY() - gridOrigin_.y) * gridResolution_);

		{
			tf::StampedTransform rotationMapCamera = transform_map_camera;
			rotationMapCamera.setOrigin(tf::Vector3(0, 0, 0));
			tf::Vector3 planeNormalCamera(plane_model.values[0], plane_model.values[1], plane_model.values[2]);
			tf::Vector3 planeNormalWorld = rotationMapCamera * planeNormalCamera;

			pcl::PointXYZRGB point = (*filtered_input_cloud)[(inliers->indices[inliers->indices.size() / 2])];
			tf::Vector3 planePointCamera(point.x, point.y, point.z);
			tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
			//std::cout << "normCam: " << planeNormalCamera.getX() << ", " << planeNormalCamera.getY() << ", " << planeNormalCamera.getZ() << "  normW: " << planeNormalWorld.getX() << ", " << planeNormalWorld.getY() << ", " << planeNormalWorld.getZ() << "   point[half]: " << planePointWorld.getX() << ", " << planePointWorld.getY() << ", " << planePointWorld.getZ() << std::endl;

			// verify that the found plane is a valid ground plane
#ifdef WITH_MAP
			if ((int)inliers->indices.size()>min_plane_points_ && planeNormalWorld.getZ()<plane_normal_max_z_ && abs(planePointWorld.getZ())<plane_max_height_)
			{
				found_plane=true;
				break;
			}
#else
			if ((int) inliers->indices.size() > min_plane_points_)
			{
				found_plane = true;
				break;
			}
#endif
			else
			{
//				// the plane is not the ground plane -> remove that plane from the point cloud
//				for (size_t i=0; i<inliers->indices.size(); i++)
//				{
//					// set all inliers to invalid
//					pcl::PointXYZRGB& point = (*filtered_input_cloud)[(inliers->indices[i])];
//					point.x = 0;
//					point.y = 0;
//					point.z = 0;
//				}

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				extract.setInputCloud(filtered_input_cloud);
				extract.setIndices(inliers);

				// Remove the planar inliers, extract the rest
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
				extract.setNegative(true);
				extract.filter(*temp);
				filtered_input_cloud = temp;
			}
		}
	}

	// if the ground plane was found, write the respective data to the images
	if (found_plane == true)
	{
		pcl::PointCloud<pcl::PointXYZRGB> floor_plane;

		// determine all point indices in original point cloud for plane inliers
		inliers->indices.clear();
		for (unsigned int i = 0; i < input_cloud->size(); i++)
		{
			pcl::PointXYZRGB& point = (*input_cloud)[i];
			if (point.x != 0. || point.y != 0. || point.z != 0.)
			{
				// check plane equation
				double distance = plane_model.values[0] * point.x + plane_model.values[1] * point.y + plane_model.values[2] * point.z + plane_model.values[3];
				if (distance > -plane_inlier_threshold && distance < plane_inlier_threshold)
					inliers->indices.push_back(i);
			}
		}

		plane_color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
		plane_mask = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC1);
		std::set<cv::Point2i, lessPoint2i> visitedGridCells; // secures that no two observations can count twice for the same grid cell
//		cv::Point2i grid_offset(0,0);	//(grid_number_observations.cols/2, grid_number_observations.rows/2);	//done: offset
		for (size_t i = 0; i < inliers->indices.size(); i++)
		{
			int v = inliers->indices[i] / input_cloud->width; // check ob das immer abrundet ->noch offen!!!
			int u = inliers->indices[i] - v * input_cloud->width;

			// cropped color image and mask
			pcl::PointXYZRGB point = (*input_cloud)[(inliers->indices[i])];
			plane_color_image.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, point.r);
			plane_mask.at<uchar>(v, u) = 255;

			// populate visibility grid
//			tf::Vector3 planePointCamera(point.x, point.y, point.z);
//			tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
//			//cv::Point2i co(-(planePointWorld.getX()-gridOrigin_.x)*gridResolution_+grid_offset.x, (planePointWorld.getY()-gridOrigin_.y)*gridResolution_+grid_offset.y);	//done: offset
//			cv::Point2i co((planePointWorld.getX() - gridOrigin_.x) * gridResolution_, (planePointWorld.getY() - gridOrigin_.y) * gridResolution_);
//			// todo: add a check whether the current point is really visible in the current image of analysis (i.e. the warped image)
//			if (visitedGridCells.find(co) == visitedGridCells.end() && co.x >= 0 && co.x < grid_number_observations.cols && co.y >= 0
//					&& co.y < grid_number_observations.rows)
//			{
//				// grid cell has not been incremented, yet
////				for (std::set<cv::Point2i, lessPoint2i>::iterator it=visitedGridCells.begin(); it!=visitedGridCells.end(); it++)
////				{
////					if (it->x==co.x && it->y==co.y)
////					{
////						std::cout << "co: " << co.x << " " << co.y << std::endl;
////						std::cout << "p:" << it->x << " " << it->y << std::endl;
////						getchar();
////					}
////				}
//				visitedGridCells.insert(co);
//				grid_number_observations.at<int>(co) = grid_number_observations.at<int>(co) + 1;
//			}

//			point.z = -(plane_model.values[0]*point.x+plane_model.values[1]*point.y+plane_model.values[3])/plane_model.values[2];
			floor_plane.push_back(point);
		}
		if (debug_["SaveDataForTest"] == true)
		{
			cv::imwrite("test_data/plane_color_image" + birdeyeresolution + ".jpg", plane_color_image);
			cv::imwrite("test_data/plane_mask" + birdeyeresolution + ".jpg", plane_mask);
		}

		//display detected floor
		//cv::imshow("floor cropped", ground_image);

		// todo: make parameter whether this should be published
		sensor_msgs::PointCloud2 floor_cloud;
		pcl::toROSMsg(floor_plane, floor_cloud);
		floor_cloud.header = header;
		floor_plane_pub_.publish(floor_cloud);

		found_plane = true;
	}

	return found_plane;
}

bool DirtDetection::computeBirdsEyePerspective(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask,
		pcl::ModelCoefficients& plane_model, cv::Mat& H, cv::Mat& R, cv::Mat& t, cv::Point2f& cameraImagePlaneOffset, cv::Mat& plane_color_image_warped,
		cv::Mat& plane_mask_warped)
{

	// 1. compute parameter representation of plane, construct plane coordinate system and compute transformation from camera frame (x,y,z) to plane frame (x,y,z)
	// a) parameter form of plane equation
	// choose two arbitrary points on the plane
	cv::Point3d p1, p2;
	double a = plane_model.values[0], b = plane_model.values[1], c = plane_model.values[2], d = plane_model.values[3];
	if (a == 0. && b == 0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 1;
		p2.y = 0;
		p2.z = -d / c;
	}
	else if (a == 0. && c == 0.)
	{
		p1.x = 0;
		p1.y = -d / b;
		p1.z = 0;
		p2.x = 1;
		p2.y = -d / b;
		p2.z = 0;
	}
	else if (b == 0. && c == 0.)
	{
		p1.x = -d / a;
		p1.y = 0;
		p1.z = 0;
		p2.x = -d / a;
		p2.y = 1;
		p2.z = 0;
	}
	else if (a == 0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 1;
		p2.y = 0;
		p2.z = -d / c;
	}
	else if (b == 0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 0;
		p2.y = 1;
		p2.z = -d / c;
	}
	else if (c == 0.)
	{
		p1.x = -d / a;
		p1.y = 0;
		p1.z = 0;
		p2.x = -d / a;
		p2.y = 0;
		p2.z = 1;
	}
	else
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 1;
		p2.y = 0;
		p2.z = (-d - a) / c;
	}
	// compute two normalized directions
	cv::Point3d dirS, dirT, normal(a, b, c);
	double lengthNormal = cv::norm(normal);
//	if (c<0.)
//		lengthNormal *= -1;
	normal.x /= lengthNormal;
	normal.y /= lengthNormal;
	normal.z /= lengthNormal;
	dirS = p2 - p1;
	double lengthS = cv::norm(dirS);
	dirS.x /= lengthS;
	dirS.y /= lengthS;
	dirS.z /= lengthS;
	dirT.x = normal.y * dirS.z - normal.z * dirS.y;
	dirT.y = normal.z * dirS.x - normal.x * dirS.z;
	dirT.z = normal.x * dirS.y - normal.y * dirS.x;
	double lengthT = cv::norm(dirT);
	dirT.x /= lengthT;
	dirT.y /= lengthT;
	dirT.z /= lengthT;

	// b) construct plane coordinate system
	// plane coordinate frame has center p1 and x-axis=dirS, y-axis=dirT, z-axis=normal

	// c) compute transformation from camera frame (x,y,z) to plane frame (x,y,z)
	t = (cv::Mat_<double>(3, 1) << p1.x, p1.y, p1.z);
	R = (cv::Mat_<double>(3, 3) << dirS.x, dirT.x, normal.x, dirS.y, dirT.y, normal.y, dirS.z, dirT.z, normal.z);

//		std::cout << "t: " << p1.x << ", " << p1.y << ", " << p1.z << std::endl;
//		std::cout << "dirS: " << dirS.x << ", " << dirS.y << ", " << dirS.z << std::endl;
//		std::cout << "dirT: " << dirT.x << ", " << dirT.y << ", " << dirT.z << std::endl;
//		std::cout << "normal: " << normal.x << ", " << normal.y << ", " << normal.z << std::endl;

	// 2. select data segment and compute final transformation of camera coordinates to scaled and centered plane coordinates
	std::vector<cv::Point2f> pointsCamera, pointsPlane;
	cv::Point2f minPlane(1e20, 1e20), maxPlane(-1e20, -1e20);
	cv::Mat RTt = R.t() * t;
	for (int v = 0; v < plane_color_image.rows; v++)
	{
		for (int u = 0; u < plane_color_image.cols; u++)
		{
			// black pixels are not part of the plane
			const cv::Vec3b& color = plane_color_image.at<cv::Vec3b>(v, u);
			if (color[0] == 0 && color[1] == 0 && color[2] == 0)
				continue;

			// distance to camera has to be below a maximum distance
			pcl::PointXYZRGB point = (*input_cloud)[v * plane_color_image.cols + u];
			if (point.x * point.x + point.y * point.y + point.z * point.z > max_distance_to_camera_ * max_distance_to_camera_)
				continue;

			// determine max and min x and y coordinates of the plane
			cv::Mat pointCamera = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
			pointsCamera.push_back(cv::Point2f(u, v));
			cv::Mat pointPlane = R.t() * pointCamera - RTt;
			pointsPlane.push_back(cv::Point2f(pointPlane.at<double>(0), pointPlane.at<double>(1)));

			if (minPlane.x > pointPlane.at<double>(0))
				minPlane.x = pointPlane.at<double>(0);
			if (maxPlane.x < pointPlane.at<double>(0))
				maxPlane.x = pointPlane.at<double>(0);
			if (minPlane.y > pointPlane.at<double>(1))
				minPlane.y = pointPlane.at<double>(1);
			if (maxPlane.y < pointPlane.at<double>(1))
				maxPlane.y = pointPlane.at<double>(1);
		}
	}

	// 3. find homography between image plane and plane coordinates
	// a) collect point correspondences
	if (pointsCamera.size() < 100)
		return false;
	double step = std::max(1.0, (double) pointsCamera.size() / 100.0);
	std::vector<cv::Point2f> correspondencePointsCamera, correspondencePointsPlane;
	cameraImagePlaneOffset = cv::Point2f((maxPlane.x + minPlane.x) / 2.f - (image_scaling_ * (double) plane_color_image.cols) / (2 * bird_eye_resolution_),
			(maxPlane.y + minPlane.y) / 2.f - (image_scaling_ * (double) plane_color_image.rows) / (2 * bird_eye_resolution_));
	for (double i = 0; i < (double) pointsCamera.size(); i += step)
	{
		correspondencePointsCamera.push_back(pointsCamera[(int) i]);
		correspondencePointsPlane.push_back(bird_eye_resolution_ * (pointsPlane[(int) i] - cameraImagePlaneOffset));
	}
	// b) compute homography
	H = cv::findHomography(correspondencePointsCamera, correspondencePointsPlane);
//		correspondencePointsCamera.push_back(cv::Point2f(160,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(320,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(480,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(160,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(320,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(480,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		for (int i=0; i<(int)correspondencePointsCamera.size(); i++)
//		{
//			cv::Mat pc = (cv::Mat_<double>(3,1) << (double)correspondencePointsCamera[i].x, (double)correspondencePointsCamera[i].y, 1.0);
//			cv::Mat pp = (cv::Mat_<double>(3,1) << (double)correspondencePointsPlane[i].x, (double)correspondencePointsPlane[i].y, 1.0);
//
//			cv::Mat Hp = H.inv()*pp;
//			Hp.at<double>(0) /= Hp.at<double>(2);
//			Hp.at<double>(1) /= Hp.at<double>(2);
//			Hp.at<double>(2) = 1.0;
//			cv::Mat r = pc - Hp;
//
//			std::cout << "H - " << i << ": " << r.at<double>(0) << ", " << r.at<double>(1) << ", " << r.at<double>(2) << std::endl;
//		}

	// 4. warp perspective
	cv::Size target_size(image_scaling_ * plane_color_image.cols, image_scaling_ * plane_color_image.rows);
	cv::warpPerspective(plane_color_image, plane_color_image_warped, H, target_size);
	// todo: better manual sampling of the warped mask needed
	cv::warpPerspective(plane_mask, plane_mask_warped, H, target_size);

//		// this example is correct, H transforms world points into the image coordinate system
//		std::vector<cv::Point2f> c1, c2;
//		c1.push_back(cv::Point2f(885,1362));
//		c2.push_back(cv::Point2f(0,0));
//		c1.push_back(cv::Point2f(880,1080));
//		c2.push_back(cv::Point2f(0, 142.7));
//		c1.push_back(cv::Point2f(945,1089));
//		c2.push_back(cv::Point2f(83.7,142.7));
//		c1.push_back(cv::Point2f(948,1350));
//		c2.push_back(cv::Point2f(83.7,0));
//		cv::Mat Hcc = cv::findHomography(c2, c1);
//		std::cout << "H: " << std::endl;
//		for (int v=0;v<3; v++)
//		{
//			for (int u=0; u<3; u++)
//				std::cout << Hcc.at<double>(v,u) << "\t";
//			std::cout << std::endl;
//		}
//		// add test points
//		c1.push_back(cv::Point2f(1010, 1338));
//		c2.push_back(cv::Point2f(167.4,0));
//		c1.push_back(cv::Point2f(1010, 1098));
//		c2.push_back(cv::Point2f(167.4,142.7));
//		for (int i=0; i<(int)c1.size(); i++)
//		{
//			cv::Mat pc = (cv::Mat_<double>(3,1) << (double)c1[i].x, (double)c1[i].y, 1.0);
//			cv::Mat pp = (cv::Mat_<double>(3,1) << (double)c2[i].x, (double)c2[i].y, 1.0);
//			cv::Mat Hccpp = Hcc*pp;
//			Hccpp.at<double>(0) /= Hccpp.at<double>(2);
//			Hccpp.at<double>(1) /= Hccpp.at<double>(2);
//			Hccpp.at<double>(2) = 1.0;
//			cv::Mat r = pc - Hccpp;
//			std::cout << "H - " << i << ": " << r.at<double>(0) << ", " << r.at<double>(1) << ", " << r.at<double>(2) << std::endl;
//		}

	return true;
}

void DirtDetection::transformPointFromCameraImageToWorld(const cv::Mat& pointCamera, const cv::Mat& H, const cv::Mat& R, const cv::Mat& t,
		const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld)
{
	cv::Mat Hp = H * pointCamera; // transformation from image plane to floor plane
	cv::Mat pointFloor = (cv::Mat_<double>(3, 1) << Hp.at<double>(0) / Hp.at<double>(2) / bird_eye_resolution_ + cameraImagePlaneOffset.x, Hp.at<double>(1)
			/ Hp.at<double>(2) / bird_eye_resolution_ + cameraImagePlaneOffset.y, 0.0);
	cv::Mat pointWorldCamera = R * pointFloor + t; // transformation from floor plane to camera world coordinates
	tf::Vector3 pointWorldCameraBt(pointWorldCamera.at<double>(0), pointWorldCamera.at<double>(1), pointWorldCamera.at<double>(2));
	tf::Vector3 pointWorldMapBt = transformMapCamera * pointWorldCameraBt;
	pointWorld.x = pointWorldMapBt.getX();
	pointWorld.y = pointWorldMapBt.getY();
	pointWorld.z = pointWorldMapBt.getZ();
}

void DirtDetection::transformPointFromCameraWarpedToWorld(const cv::Mat& pointPlane, const cv::Mat& R, const cv::Mat& t,
		const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld)
{
	tf::Vector3 pointWorldMapBt;
	if (warp_image_ == true)
	{
		cv::Mat pointFloor = (cv::Mat_<double>(3, 1) << pointPlane.at<double>(0) / bird_eye_resolution_ + cameraImagePlaneOffset.x, pointPlane.at<double>(1)
				/ bird_eye_resolution_ + cameraImagePlaneOffset.y, 0.0);
		cv::Mat pointWorldCamera = R * pointFloor + t; // transformation from floor plane to camera world coordinates
		tf::Vector3 pointWorldCameraBt(pointWorldCamera.at<double>(0), pointWorldCamera.at<double>(1), pointWorldCamera.at<double>(2));
		pointWorldMapBt = transformMapCamera * pointWorldCameraBt;
	}
	else
	{
		tf::Vector3 pointWorldCameraBt(pointPlane.at<double>(0), pointPlane.at<double>(1), pointPlane.at<double>(2));
		pointWorldMapBt = transformMapCamera * pointWorldCameraBt;
	}
	pointWorld.x = pointWorldMapBt.getX();
	pointWorld.y = pointWorldMapBt.getY();
	pointWorld.z = pointWorldMapBt.getZ();
}

void DirtDetection::transformPointFromWorldToCameraWarped(const cv::Point3f& pointWorld, const cv::Mat& R, const cv::Mat& t,
		const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Mat& pointPlane)
{
	tf::Vector3 pointWorldMapBt(pointWorld.x, pointWorld.y, pointWorld.z);
	tf::Vector3 pointWorldCameraBt;
	pointPlane.create(3, 1, CV_64FC1);
	if (warp_image_ == true)
	{
		pointWorldCameraBt = transformMapCamera.inverse() * pointWorldMapBt;
		cv::Mat pointWorldCamera = (cv::Mat_<double>(3, 1) << pointWorldCameraBt.getX(), pointWorldCameraBt.getY(), pointWorldCameraBt.getZ());
		cv::Mat pointFloor = R.t() * (pointWorldCamera - t); // transformation from camera world coordinates to floor plane
		pointPlane.at<double>(0) = (pointFloor.at<double>(0) - cameraImagePlaneOffset.x) * bird_eye_resolution_;
		pointPlane.at<double>(1) = (pointFloor.at<double>(1) - cameraImagePlaneOffset.y) * bird_eye_resolution_;
		pointPlane.at<double>(2) = 1.;
	}
	else
	{
		tf::Vector3 pointWorldCameraBt(pointPlane.at<double>(0), pointPlane.at<double>(1), pointPlane.at<double>(2));
		pointWorldCameraBt = transformMapCamera.inverse() * pointWorldMapBt;
		pointPlane.at<double>(0) = pointWorldCameraBt.getX();
		pointPlane.at<double>(1) = pointWorldCameraBt.getY();
		pointPlane.at<double>(2) = pointWorldCameraBt.getZ();
	}
}

void DirtDetection::SaliencyDetection_C1(const cv::Mat& C1_image, cv::Mat& C1_saliency_image)
{
	//given a one channel image
	//int scale = 6;
	//unsigned int size = (int)floor((float)pow(2.0,scale)); //the size to do the saliency at
	unsigned int size_cols = (int) (C1_image.cols * spectral_residual_image_size_ratio_);
	unsigned int size_rows = (int) (C1_image.rows * spectral_residual_image_size_ratio_);

	//create different images
	cv::Mat bw_im;
	cv::resize(C1_image, bw_im, cv::Size(size_cols, size_rows));

	cv::Mat realInput(size_rows, size_cols, CV_32FC1); //calculate number of test samples
	//int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	//printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
	cv::Mat imaginaryInput(size_rows, size_cols, CV_32FC1);
	cv::Mat complexInput(size_rows, size_cols, CV_32FC2);

	bw_im.convertTo(realInput, CV_32F, 1.0 / 255, 0);
	imaginaryInput = cv::Mat::zeros(size_rows, size_cols, CV_32F);

	std::vector<cv::Mat> vec;
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, complexInput);

	cv::Mat dft_A(size_rows, size_cols, CV_32FC2);

	cv::dft(complexInput, dft_A, cv::DFT_COMPLEX_OUTPUT, size_rows);
	vec.clear();
	cv::split(dft_A, vec);
	realInput = vec[0];
	imaginaryInput = vec[1];

	// Compute the phase angle
	cv::Mat image_Mag(size_rows, size_cols, CV_32FC1);
	cv::Mat image_Phase(size_rows, size_cols, CV_32FC1);

	//compute the phase of the spectrum
	cv::cartToPolar(realInput, imaginaryInput, image_Mag, image_Phase, 0);
//	std::vector<DirtDetection::CarpetFeatures> test_feat_vec;
//	std::vector<DirtDetection::CarpetClass> test_class_vec;

	std::string name;
	std::vector<std::string> name_vec;

	name = "carpet-gray.tepp";
	name_vec.push_back(name);
	cv::Mat log_mag(size_rows, size_cols, CV_32FC1);
	cv::log(image_Mag, log_mag);

//	cv::Mat log_mag_;
//	cv::normalize(log_mag, log_mag_, 0, 1, NORM_MINMAX);
//	cv::imshow("log_mag", log_mag_);

	//Box filter the magnitude, then take the difference
	cv::Mat log_mag_Filt(size_rows, size_cols, CV_32FC1);

	cv::Mat filt = cv::Mat::ones(3, 3, CV_32FC1) * 1. / 9.;
	//filt.convertTo(filt,-1,1.0/9.0,0);

	cv::filter2D(log_mag, log_mag_Filt, -1, filt);
	//cv::GaussianBlur(log_mag, log_mag_Filt, cv::Size2i(25,25), 0);
	//log_mag = log_mag_Filt.clone();
	//cv::medianBlur(log_mag, log_mag_Filt, 5);

//	cv::Mat log_mag_filt_;
//	cv::normalize(log_mag_Filt, log_mag_filt_, 0, 1, NORM_MINMAX);
//	cv::imshow("log_mag_filt", log_mag_filt_);

	//cv::subtract(log_mag, log_mag_Filt, log_mag);
	log_mag -= log_mag_Filt;

//	cv::Mat log_mag_sub_a_, log_mag_sub_;
//	cv::normalize(log_mag, log_mag_sub_a_, 0, 1, NORM_MINMAX);
//	cv::GaussianBlur(log_mag_sub_a_, log_mag_sub_, cv::Size2i(21,21), 0);
//	cv::imshow("log_mag_sub", log_mag_sub_);
//	log_mag_Filt = log_mag.clone();
//	cv::GaussianBlur(log_mag_Filt, log_mag, cv::Size2i(21,21), 0);
//	void GBTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
//						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
//						CvGBTrees &carpet_GBTree);
	cv::exp(log_mag, image_Mag);

	cv::polarToCart(image_Mag, image_Phase, realInput, imaginaryInput, 0);

	vec.clear();
	vec.push_back(realInput);
	vec.push_back(imaginaryInput);
	cv::merge(vec, dft_A);

	cv::dft(dft_A, dft_A, cv::DFT_INVERSE, size_rows);

	dft_A = abs(dft_A);
	dft_A.mul(dft_A);

	cv::split(dft_A, vec);

	C1_saliency_image = vec[0];
	std::cout << "The size of the output of 1 channel fft is: " << C1_saliency_image.cols << " " << C1_saliency_image.rows << " "
			<< C1_saliency_image.channels() << std::endl;
}

void DirtDetection::SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask, int gaussianBlurCycles)
{
	cv::Mat fci; // "fci"<-> first channel image
	cv::Mat sci; // "sci"<-> second channel image
	cv::Mat tci; //"tci"<-> second channel image

	std::vector<cv::Mat> vec;
	vec.push_back(fci);
	vec.push_back(sci);
	vec.push_back(tci);

	cv::split(C3_color_image, vec);

	fci = vec[0];
	sci = vec[1];
	tci = vec[2];

	cv::Mat res_fci; // "fci"<-> first channel image
	cv::Mat res_sci; // "sci"<-> second channel image
	cv::Mat res_tci; //"tci"<-> second channel image

	SaliencyDetection_C1(fci, res_fci);
	SaliencyDetection_C1(sci, res_sci);
	SaliencyDetection_C1(tci, res_tci);

	cv::Mat realInput;

	realInput = (res_fci + res_sci + res_tci) / 3;
	std::cout << "The fft output size after 1 channel is: " << realInput.cols << " " << realInput.rows << std::endl;

	cv::Size2i ksize;
	ksize.width = 3;
	ksize.height = 3;
	for (int i = 0; i < gaussianBlurCycles; i++)
		cv::GaussianBlur(realInput, realInput, ksize, 0); //necessary!? --> less noise

	cv::resize(realInput, C1_saliency_image, C3_color_image.size());
	std::cout << "The fft output size after resize is: " << C1_saliency_image.cols << " " << C1_saliency_image.rows << std::endl;

	// remove borders of the ground plane because of artifacts at the border like lines
	if (mask != 0)
	{
		// maske erodiere
		cv::Mat mask_eroded = mask->clone();
		cv::dilate(*mask, mask_eroded, cv::Mat(), cv::Point(-1, -1), 2);
		cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 25.0 / 640.0 * C3_color_image.cols);
		cv::Mat neighborhood = cv::Mat::ones(3, 1, CV_8UC1);
		cv::erode(mask_eroded, mask_eroded, neighborhood, cv::Point(-1, -1), 15.0 / 640.0 * C3_color_image.cols);

		//cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 35.0/640.0*C3_color_image.cols);
		// todo: hack for autonomik
		//cv::erode(mask_eroded, mask_eroded, cv::Mat(), cv::Point(-1, -1), 45.0/640.0*C3_color_image.cols);

		cv::Mat temp;
		C1_saliency_image.copyTo(temp, mask_eroded);
		C1_saliency_image = temp;
	}

	// remove saliency at the image border (because of artifacts in the corners)
	int borderX = C1_saliency_image.cols / 20;
	int borderY = C1_saliency_image.rows / 20;
	if (borderX > 0 && borderY > 0)
	{
		cv::Mat smallImage_ = C1_saliency_image.colRange(borderX, C1_saliency_image.cols - borderX);
		cv::Mat smallImage = smallImage_.rowRange(borderY, smallImage_.rows - borderY);

		copyMakeBorder(smallImage, C1_saliency_image, borderY, borderY, borderX, borderX, cv::BORDER_CONSTANT, cv::Scalar(0));
	}

	// display the individual channels
//	for (int i=0; i<gaussianBlurCycles; i++)
//		cv::GaussianBlur(res_fci, res_fci, ksize, 0); //necessary!? --> less noise
//	for (int i=0; i<gaussianBlurCycles; i++)
//		cv::GaussianBlur(res_sci, res_sci, ksize, 0); //necessary!? --> less noise	//calculate number of test samples
//	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
//	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);
//	for (int i=0; i<gaussianBlurCycles; i++)
//		cv::GaussianBlur(res_tci, res_tci, ksize, 0); //necessary!? --> less noise
//
//	cv::resize(res_fci,res_fci,C3_color_image.size());
//	cv::resize(res_sci,res_sci,C3_color_image.size());
//	cv::resize(res_tci,res_tci,C3_color_image.size());
//
//	// scale input_image
//	double minv, maxv;
//	cv::Point2i minl, maxl;
//	cv::minMaxLoc(res_fci,&minv,&maxv,&minl,&maxl);
//	res_fci.convertTo(res_fci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	cv::minMaxLoc(res_sci,&minv,&maxv,&minl,&maxl);
//	res_sci.convertTo(res_sci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	cv::minMaxLoc(res_tci,&minv,&maxv,&minl,&maxl);
//	res_tci.convertTo(res_tci, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//
//	cv::imshow("b", res_fci);
//	cv::imshow("g", res_sci);
//	cv::imshow("r", res_tci);
}

void DirtDetection::Image_Postprocessing_C1(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image)
{
	// scale input_image
	cv::Mat scaled_input_image;
	double minv, maxv;
	cv::Point2i minl, maxl;
	cv::minMaxLoc(C1_saliency_image, &minv, &maxv, &minl, &maxl);
	C1_saliency_image.convertTo(scaled_input_image, -1, 1.0 / (maxv - minv), 1.0 * (minv) / (maxv - minv));

	if (debug_["showSaliencyDetection"] == true)
	{
		cv::imshow("saliency detection", scaled_input_image);
		cvMoveWindow("saliency detection", 0, 530);
	}

	//set dirt pixel to white
	C1_BlackWhite_image = cv::Mat::zeros(C1_saliency_image.size(), CV_8UC1);
	cv::threshold(scaled_input_image, C1_BlackWhite_image, dirt_threshold_, 1, cv::THRESH_BINARY);

//	std::cout << "(C1_saliency_image channels) = (" << C1_saliency_image.channels() << ")" << std::endl;

	cv::Mat CV_8UC_image;
	C1_BlackWhite_image.convertTo(CV_8UC_image, CV_8UC1);

//	Mat dst = Mat::zeros(img.rows, img.cols, CV_8UC3);
//	dst = C3_color_image;

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(CV_8UC_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	cv::Scalar color(0, 255, 0);
	cv::RotatedRect rec;

	for (int i = 0; i < (int) contours.size(); i++)
	{
		rec = cv::minAreaRect(contours[i]);
		cv::ellipse(C3_color_image, rec, color, 2);
	} //calculate number of test samples
	int NumTestSamples = 10; //ceil(NumSamples*percentage_testdata);
	printf("Anzahl zu ziehender test samples: %d \n", NumTestSamples);

}

void DirtDetection::Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image,
		std::vector<cv::RotatedRect>& dirtDetections, const cv::Mat& mask)
{
	// dirt detection on image with artificial dirt
	cv::Mat color_image_with_artifical_dirt = C3_color_image.clone();
	cv::Mat mask_with_artificial_dirt = mask.clone();
	// add dirt
	int dirtSize = cvRound(3.0 / 640.0 * C3_color_image.cols);
	cv::Point2f ul(0.4375 * C3_color_image.cols, 0.416666667 * C3_color_image.rows);
	cv::Point2f ur(0.5625 * C3_color_image.cols, 0.416666667 * C3_color_image.rows);
	cv::Point2f ll(0.4375 * C3_color_image.cols, 0.583333333 * C3_color_image.rows);
	cv::Point2f lr(0.5625 * C3_color_image.cols, 0.583333333 * C3_color_image.rows);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ul, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ul, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(lr, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(lr, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ll, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ll, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::ellipse(color_image_with_artifical_dirt, cv::RotatedRect(ur, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(0, 0, 0), dirtSize);
	cv::ellipse(mask_with_artificial_dirt, cv::RotatedRect(ur, cv::Size2f(dirtSize, dirtSize), 0), cv::Scalar(255, 255, 255), dirtSize);
	cv::Mat C1_saliency_image_with_artifical_dirt;
	SaliencyDetection_C3(color_image_with_artifical_dirt, C1_saliency_image_with_artifical_dirt, &mask_with_artificial_dirt,
			spectral_residual_gaussian_blur_iterations_);
	//cv::imshow("ai_dirt", color_image_with_artifical_dirt);

	// display of images with artificial dirt
	if (debug_["showColorWithArtificialDirt"] == true)
		cv::imshow("color with artificial dirt", color_image_with_artifical_dirt);
	cv::Mat C1_saliency_image_with_artifical_dirt_scaled;
	double salminv, salmaxv;
	cv::Point2i salminl, salmaxl;
	cv::minMaxLoc(C1_saliency_image_with_artifical_dirt, &salminv, &salmaxv, &salminl, &salmaxl, mask_with_artificial_dirt);
	C1_saliency_image_with_artifical_dirt.convertTo(C1_saliency_image_with_artifical_dirt_scaled, -1, 1.0 / (salmaxv - salminv),
			-1.0 * (salminv) / (salmaxv - salminv));
	if (debug_["showSaliencyWithArtificialDirt"] == true)
		cv::imshow("saliency with artificial dirt", C1_saliency_image_with_artifical_dirt_scaled);

	// scale C1_saliency_image to value obtained from C1_saliency_image with artificially added dirt
//	std::cout << "res_img: " << C1_saliency_image_with_artifical_dirt.at<float>(300,200);
//	C1_saliency_image_with_artifical_dirt = C1_saliency_image_with_artifical_dirt.mul(C1_saliency_image_with_artifical_dirt);	// square C1_saliency_image_with_artifical_dirt to emphasize the dirt and increase the gap to background response
//	std::cout << " -> " << C1_saliency_image_with_artifical_dirt.at<float>(300,200) << std::endl;
	double minv, maxv;
	cv::Point2i minl, maxl;
	cv::minMaxLoc(C1_saliency_image_with_artifical_dirt, &minv, &maxv, &minl, &maxl, mask_with_artificial_dirt);
	cv::Scalar mean, stdDev;
	cv::meanStdDev(C1_saliency_image_with_artifical_dirt, mean, stdDev, mask);
	double newMaxVal = std::min(1.0, maxv / spectral_residual_normalization_highest_max_value_); ///mean.val[0] / spectralResidualNormalizationHighestMaxMeanRatio_);
//	std::cout << "dirtThreshold=" << dirtThreshold_ << "\tmin=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << "\tnewMaxVal (r)=" << newMaxVal << std::endl;

	//determine ros package path
	// todo: learning part
//	std::string svmpath = ros::package::getPath("ipa_dirt_detection") + "/common/files/svm/Teppich1.tepp";
//	ofstream teppichfile;
//	teppichfile.open (svmpath.c_str(), ios::out| ios::app);
//	teppichfile << dirtThreshold_ << "\t\t" << minv << "\t\t" << maxv << "\t\t" << mean.val[0] << "\t\t" << stdDev.val[0] << "\n";
//	teppichfile.close();

	////C1_saliency_image.convertTo(scaled_input_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
	cv::Mat scaled_C1_saliency_image = C1_saliency_image.clone(); // square C1_saliency_image_with_artifical_dirt to emphasize the dirt and increase the gap to background response
	//scaled_C1_saliency_image = scaled_C1_saliency_image.mul(scaled_C1_saliency_image);
	scaled_C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal / (maxv - minv), -newMaxVal * (minv) / (maxv - minv));

	double newMean = mean.val[0] * newMaxVal / (maxv - minv) - newMaxVal * (minv) / (maxv - minv);
	double newStdDev = stdDev.val[0] * newMaxVal / (maxv - minv);
//	std::cout << "newMean=" << newMean << "   newStdDev=" << newStdDev << std::endl;

//	// scale C1_saliency_image
//	cv::Mat scaled_C1_saliency_image;
//	double minv, maxv;
//	cv::Point2i minl, maxl;
//	cv::minMaxLoc(C1_saliency_image,&minv,&maxv,&minl,&maxl, mask);
	cv::Mat badscale;
	double badminv, badmaxv;
	cv::Point2i badminl, badmaxl;
	cv::minMaxLoc(C1_saliency_image, &badminv, &badmaxv, &badminl, &badmaxl, mask);
	C1_saliency_image.convertTo(badscale, -1, 1.0 / (badmaxv - badminv), -1.0 * (badminv) / (badmaxv - badminv));
//	std::cout << "bad scale:   " << "\tmin=" << badminv << "\tmax=" << badmaxv << std::endl;
	if (debug_["showSaliencyBadScale"] == true)
	{
		cv::imshow("bad scale", badscale);
		cvMoveWindow("bad scale", 650, 0);
	}
	//cvMoveWindow("bad scale", 650, 520);
//	cv::Scalar mean, stdDev;
//	cv::meanStdDev(C1_saliency_image, mean, stdDev, mask);
//	std::cout << "min=" << minv << "\tmax=" << maxv << "\tmean=" << mean.val[0] << "\tstddev=" << stdDev.val[0] << std::endl;
//
//	double newMaxVal = min(1.0, maxv/mean.val[0] /5.);
//	//C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, 1.0/(maxv-minv), 1.0*(minv)/(maxv-minv));
//	C1_saliency_image.convertTo(scaled_C1_saliency_image, -1, newMaxVal/(maxv-minv), -newMaxVal*(minv)/(maxv-minv));

	// remove responses that lie on lines
	if (remove_lines_ == true)
	{
		cv::Mat src, dst, color_dst;

		cv::cvtColor(C3_color_image, src, CV_BGR2GRAY);

		// hack: autonomik
		//cv::Canny(src, dst, 150, 200, 3);
		cv::Canny(src, dst, 90, 170, 3);
		cv::cvtColor(dst, color_dst, CV_GRAY2BGR);

		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(dst, lines, 1, CV_PI / 180, 80, 30, 10);
		for (size_t i = 0; i < lines.size(); i++)
		{
			line(color_dst, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 255), 3, 8);
			line(scaled_C1_saliency_image, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 0, 0), 13, 8);
			// todo: hack for autonomik
			//line(scaled_C1_saliency_image, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,0), 35, 8);
		}

		if (debug_["showDetectedLines"] == true)
		{
			cv::namedWindow("Detected Lines", 1);
			cv::imshow("Detected Lines", color_dst);
		}
	}

	if (debug_["showSaliencyDetection"] == true)
	{
		cv::imshow("saliency detection", scaled_C1_saliency_image);
		cvMoveWindow("saliency detection", 0, 530);
	}

	//set dirt pixel to white
	C1_BlackWhite_image = cv::Mat::zeros(C1_saliency_image.size(), CV_8UC1);
	cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, dirt_threshold_, 1, cv::THRESH_BINARY);
//	cv::threshold(scaled_C1_saliency_image, C1_BlackWhite_image, mean.val[0] + stdDev.val[0] * dirtCheckStdDevFactor_, 1, cv::THRESH_BINARY);

//	std::cout << "(C1_saliency_image channels) = (" << C1_saliency_image.channels() << ")" << std::endl;

	cv::Mat CV_8UC_image;
	C1_BlackWhite_image.convertTo(CV_8UC_image, CV_8UC1);

//	Mat dst = Mat::zeros(img.rows, img.cols, CV_8UC3);
//	dst = C3_color_image;

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(CV_8UC_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	cv::Scalar green(0, 255, 0);
	cv::Scalar red(0, 0, 255);
	for (int i = 0; i < (int) contours.size(); i++)
	{
		cv::RotatedRect rec = minAreaRect(contours[i]);
		double meanIntensity = 0;
		for (int t = 0; t < (int) contours[i].size(); t++)
		{
			meanIntensity += scaled_C1_saliency_image.at<float>(contours[i][t].y, contours[i][t].x);
			int basescale_x = (int) 1. / image_scaling_ * contours[i][t].x;
			int basescale_y = (int) 1. / image_scaling_ * contours[i][t].y;
			multiscale_scores_.at<float>(basescale_y, basescale_x) += 1.;
		}
		meanIntensity /= (double) contours[i].size();
		if (meanIntensity > newMean + dirt_check_std_dev_factor_ * newStdDev)
		{
			// todo: hack: for autonomik only detect green ellipses
			//dirtDetections.push_back(rec);
			cv::ellipse(C3_color_image, rec, green, 2);
		}
		else
			cv::ellipse(C3_color_image, rec, green, 2); // todo: use red
		dirtDetections.push_back(rec);
	}
	cv::imshow("MultiscaleScores", multiscale_scores_ * 255.);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dirt_detection");

	ros::NodeHandle n;

	DirtDetection id(n);
	id.init();

	ros::spin();

	return 0;
}

