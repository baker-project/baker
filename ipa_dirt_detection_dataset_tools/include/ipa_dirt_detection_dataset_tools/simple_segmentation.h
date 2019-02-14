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
	void segment(const cv::Mat& image);
	void segement_singlechannel();
	void crop();
	void examine();
private:

	std::string source_image_path_;
	std::string cropped_image_path_;
	std::string cropped_mask_path_;
	int crop_residual_;

	cv::Mat dirt_frame_, cropped_dirt_frame_;
	cv::Mat mask_frame_, cropped_mask_frame_;

};
};

#endif
