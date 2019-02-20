#ifndef IMAGE_BLEND_H
#define IMAGE_BLEND_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <ctime>
#include <dirent.h>
#include <math.h>
#include <vector>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/algorithm/string.hpp>

namespace ipa_dirt_detection_dataset_tools
{
class ImageBlender
{
public:
	ImageBlender(const std::string& clean_ground_path, const std::string& segmented_dirt_path, const std::string& segmented_dirt_mask_path,
			const std::string& segmented_objects_path, const std::string& segmented_objects_mask_path, const std::string& brightness_shadow_mask_path,
			const std::string& blended_img_folder, const std::string& blended_mask_folder, const std::string& blended_img_bbox_filename, const int max_num_dirt,
			const int min_num_dirt, const int max_num_objects, const int min_num_objects, const bool flip_clean_ground, const int ground_image_reuse_times);
	~ImageBlender();

	void run();

	// creates lists of all image files (clean images, dirt images and masks, object images, illumination and shadow images)
	void collectImageFiles();

	// blend dirt samples into the image
	void blendImageDirt(cv::Mat& blended_image, cv::Mat& blended_mask, const int dirt_num, std::ofstream& bbox_labels_file, const std::string& base_filename);

	// blend object samples into the image
	void blendImageObjects(cv::Mat& blended_image, cv::Mat& blended_mask, const int object_num, std::ofstream& bbox_labels_file, const std::string& base_filename);

	// rotates the image and mask with random rotation
	void rotateImage(cv::Mat& image, cv::Mat& image_mask, const double scale_factor=1., const int interpolation_mode=CV_INTER_LINEAR);

	// reduces an image's bounding box to the area of the mask
	void shrinkBoundingBox(cv::Mat& image, cv::Mat& image_mask);

	// the function is used for extracting the class name from the file name of the segmented patches
	void getPatchClassname(const std::string& patch_name, std::string& class_name);

	void edge_smoothing(cv::Mat& blended_image, cv::Mat& blended_mask, const int half_kernel_size);
	void shadow_and_illuminance(cv::Mat& blended_image, const bool shadow_or_illuminance);
	void shadow_and_illuminance_new(cv::Mat& blended_image, const bool shadow_or_illuminance);

	void resizeDirt(cv::Mat& dirt_image, cv::Mat& dirt_mask);

private:
	std::vector<std::string> clean_ground_filenames_;
	std::vector<std::string> segmented_dirt_filenames_;
	std::vector<std::string> segmented_dirt_mask_filenames_;
	std::vector<std::string> segmented_objects_filenames_;
	std::vector<std::string> segmented_objects_mask_filenames_;
	std::vector<std::string> brightness_shadow_mask_filenames_;

	int num_clean_ground_images_;
	int num_segmented_dirt_images_;
	int num_segmented_object_images_;
	int num_brightness_shadow_mask_images_;


	// parameters
	std::string clean_ground_path_;				// clean ground images path
	std::string segmented_dirt_path_;			// path to the segmented dirt samples
	std::string segmented_dirt_mask_path_;		// path to the segmented dirt masks
	std::string segmented_objects_path_;		// path to the segmented object samples
	std::string segmented_objects_mask_path_;	// path to the segmented object masks
	std::string brightness_shadow_mask_path_;	// path to brightness and shadow masks, source folder for illumination and shadow masks

	std::string blended_img_folder_;			// path to save the blended images
	std::string blended_mask_folder_;			// path to save the blended image masks
	std::string blended_img_bbox_filename_;		// name to the output file which stores the parameters of the bounding boxes of the blended images

	int max_num_dirt_;		// maximum number of dirt spots per frame
	int min_num_dirt_;		// minimum number of dirt spots per frame

	int max_num_objects_;	// maximum number of objects per frame
	int min_num_objects_;	// minimum number of objects per frame

	bool flip_clean_ground_;		// option, whether to flip the clean ground images horizontally and vertically or not (generates 4 images out of one ground image), false=off, true=on
	int ground_image_reuse_times_;	// number of reuses for the same ground pattern (i.e. how many times each image will be used for blending artificial images)
};
template<typename T>
std::string to_string(T Number)
{
	std::ostringstream ss;
	ss << Number;
	return ss.str();
}
}
;

#endif
