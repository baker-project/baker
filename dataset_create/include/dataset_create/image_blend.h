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

namespace DatasetCreate
{
class ImageBlend
{
public:
	ImageBlend(std::string& clean_ground_path, std::string& artificial_dirt_path, std::string& artificial_dirt_mask_path, std::string& segmented_pens_path,
			std::string& segmented_pens_mask_path, std::string& blended_img_folder, std::string& blended_mask_folder, int max_num_dirt, int min_num_dirt, int max_num_pens,
			int min_num_pens, std::string& filename_bbox, bool flip_clean_ground_, std::string& brightness_shadow_mask_path);
	~ImageBlend();

	void rotateImage();
	void resize_dirt();
	void blendImage(int dirt_num, std::ofstream& myfile, std::string& ground_image_name);
	void blendImage(int dirt_num, std::ofstream& myfile, std::string& ground_image_name, bool if_for_classification);
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

	int max_num_dirt_;
	int min_num_dirt_;

	int max_num_pens_;
	int min_num_pens_;

	bool flip_clean_ground_;
	bool if_resize_dirt_;
	float resize_ratio_;

	int serie_num_;    // the index to distinguish the same ground image of diiferent poses or after reusing

	cv::Mat clean_ground_pattern_;
	cv::Mat artificial_dirt_;
	cv::Mat artificial_dirt_mask_;

	cv::Mat rotated_artificial_dirt_;
	cv::Mat rotated_artificial_dirt_mask_;

	cv::Mat rotated_schranked_artificial_dirt_;
	cv::Mat rotated_shranked_artificial_dirt_mask_;

	cv::Mat fliped_ground_img_;

	std::string clean_ground_path_;
	std::string artificial_dirt_path_;
	std::string artificial_dirt_mask_path_;
	std::string segmented_pens_path_;
	std::string segmented_pens_mask_path_;
	std::string brightness_shadow_mask_path_;

	std::string blended_img_folder_;
	std::string blended_mask_folder_;
	std::string filename_bbox_;

	std::vector<std::string> clean_ground_filenames_;
	std::vector<std::string> artificial_dirt_filenames_;
	std::vector<std::string> artificial_dirt_mask_filenames_;
	std::vector<std::string> brightness_shadow_mask_filenames_;

	std::vector<std::string> artificial_pens_filenames_;
	std::vector<std::string> artificial_pens_mask_filenames_;

	int num_clean_ground_images_, num_artificial_dirt_images_;
	int num_pens_images_;

};
template<typename T>
std::string to_string(T Number)
{
	std::ostringstream ss;
	ss << Number;
	return ss.str();
}
};

#endif
