#pragma once

//STANDARD
#include <iostream>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>

//OPENCV
#include <opencv2/opencv.hpp>

//MESSAGES
#include <baker_msgs/DirtDetectionAction.h>
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ipa_dirt_detection/DirtDetectionSpectralConfig.h>

//BOOST
#include <boost/thread/mutex.hpp>

//TF
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


namespace IpaDirtDetection
{ //starting namespace

class ServerDirtDetection
{ //starting class
public:
	//Constructor
	ServerDirtDetection(ros::NodeHandle node_handle);
	//Destructor
	~ServerDirtDetection();

	//================ Functions ==================================================================================
	//=============================================================================================================

	/**
	 * This function does the action on the action server and communicates with the client.
	 *
	 * @param [in] goal			Gets this goal from the action client in a message.
	 * @param [in] as			Action server: Node to communicate with client.
	 */
	void dirtDetectionActionCallback(const baker_msgs::DirtDetectionGoalConstPtr& goal);

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
	 * @param [out]	C1_saliency_image		One channel image('C1') which results from the saliency detection.
	 * @param [in] 	mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 * @param [in]	gaussianBlurCycles		Determines the number of repetitions of the gaussian filter used to reduce the noise.
	 */
	void SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask = 0, int gaussianBlurCycles = 2);

	/**
	 *
	 * This function uses the "C1_saliency_image" to mark the dirt in the "C3_color_image".
	 * It also avoids false detections if no dirt is on the floor.
	 * Furthermore, it returns an image ("C1_BlackWhite_image") in which all dirt pixels are white (255) and all other pixels are black (0).
	 *
	 * @param [in] 		C1_saliency_image		One channel('C1') saliency image used for postprocessing.
	 * @param [in]		mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 * @param [in,out] 	C3_color_image			Three channel('C3') color which corresponds to the "C1_saliency_image".
	 * @param [out]		C1_BlackWhite_image		One channel('C1') image in which all dirt pixels are white (255) and all other pixels are black (0).
	 *
	 */
	void Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image,
			std::vector<cv::RotatedRect>& dirt_detections, cv::Mat& dirt_detections_mask, const cv::Mat& mask = cv::Mat());

protected:
	bool convertImageMessageToMat(const sensor_msgs::Image& image_msg, cv_bridge::CvImagePtr& image_ptr, cv::Mat& image);

	void dynamicReconfigureCallback(ipa_dirt_detection::DirtDetectionSpectralConfig &config, uint32_t level);

	/// dynamic reconfigure
	dynamic_reconfigure::Server<ipa_dirt_detection::DirtDetectionSpectralConfig> dynamic_reconfigure_server_;

	//ROS node handle
	ros::NodeHandle node_handle_;

	tf::TransformListener transform_listener_;

	//Action server
	actionlib::SimpleActionServer<baker_msgs::DirtDetectionAction> dirt_detection_action_server_;

	//PARAMS
	int spectral_residual_gaussian_blur_iterations_;
	double spectral_residual_normalization_highest_max_value_;
	double spectral_residual_image_size_ratio_;
	bool remove_lines_; // if true, strong lines in the image will not produce dirt responses
	double dirt_threshold_;
	double dirt_check_std_dev_factor_;


	std::map<std::string, bool> debug_;
}; //ending class

}; //ending namespace
