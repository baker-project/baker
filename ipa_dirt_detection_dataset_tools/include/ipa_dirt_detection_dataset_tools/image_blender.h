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

	// creates lists of all image files (clean images, dirt images and masks, object images, illumination and shadow images)
	void collectImageFiles();

	void rotateImage();
	void resize_dirt();
	void blendImage(int dirt_num, std::ofstream& myfile, const std::string& ground_image_name);
	void blendImage(int dirt_num, std::ofstream& myfile, const std::string& ground_image_name, bool if_for_classification);
	void shrank_bounding_box();
	void get_patch_classname(std::string& patch_name, std::string& class_name);
	void run();
	void edge_smoothing(int half_kernel_size);
	void shadow_and_illuminance(bool shadow_or_illuminance);
	void shadow_and_illuminance_new(bool shadow_or_illuminance);

	cv::Mat blended_img_;
	cv::Mat blended_mask_;
private:
	int img_cols_;
	int img_rows_;

	bool if_resize_dirt_;
	float resize_ratio_;

	int serie_num_;    // the index to distinguish the same ground image of different poses or after reusing

	cv::Mat clean_ground_pattern_;
	cv::Mat artificial_dirt_;
	cv::Mat artificial_dirt_mask_;

	cv::Mat rotated_artificial_dirt_;
	cv::Mat rotated_artificial_dirt_mask_;

	cv::Mat rotated_schranked_artificial_dirt_;
	cv::Mat rotated_shranked_artificial_dirt_mask_;

	cv::Mat fliped_ground_img_;

	std::vector<std::string> clean_ground_filenames_;
	std::vector<std::string> segmented_dirt_filenames_;
	std::vector<std::string> segmented_dirt_mask_filenames_;
	std::vector<std::string> segmented_objects_filenames_;
	std::vector<std::string> segmented_objects_mask_filenames_;
	std::vector<std::string> brightness_shadow_mask_filenames_;

	int num_clean_ground_images_;
	int num_segmented_dirt_images_;
	int num_object_images_;


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

	bool flip_clean_ground_;		// option, whether to flip the clean ground images horizontally and vertically or not (generates 4 images out of one ground image), False=off, True=on
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
