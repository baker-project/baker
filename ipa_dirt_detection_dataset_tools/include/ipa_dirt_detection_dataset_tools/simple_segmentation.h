#ifndef SIMPLE_SEGMENT_H
#define SIMPLE_SEGMENT_H

#include <iostream>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/algorithm/string.hpp>
#include <valarray>

namespace ipa_dirt_detection_dataset_tools
{
class SimpleSegmentation
{
public:
	SimpleSegmentation(const std::string dirt_image_path, const std::string cropped_image_path, const std::string cropped_mask_path, const int crop_residual);
	~SimpleSegmentation();

	void run();

	// find rectangular background
	cv::RotatedRect findRectangularBackground(const cv::Mat& image);

	// crop original image to inner axis aligned rectangle of target rectangular background
	void removeUncontrolledBackground(const cv::Mat& src_image, cv::Mat& cropped_image, const cv::RotatedRect& foreground_rectangle);

	void segment(const cv::Mat& image, cv::Mat& mask_frame);

	void crop(const cv::Mat& dirt_frame, const cv::Mat& mask_frame, cv::Mat& cropped_dirt_frame, cv::Mat& cropped_mask_frame);

	void examine(const cv::Mat& dirt_frame, const cv::Mat& mask_frame);
private:

	// parameters
	std::string source_image_path_;
	std::string cropped_image_path_;
	std::string cropped_mask_path_;
	int crop_residual_;
};
};

#endif
