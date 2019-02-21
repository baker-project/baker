#include <ros/ros.h>
#include <string>
#include "ipa_dirt_detection_dataset_tools/image_blender.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_blender");
	ros::NodeHandle pnh("~");

	// read parameters
	std::string base_path;
	pnh.param("base_path", base_path, std::string(""));
	std::cout << "base_path: " << base_path << std::endl;
	std::string ground_image_path;
	pnh.param("ground_image_path", ground_image_path, std::string("floors"));
	std::cout << "ground_image_path: " << ground_image_path << std::endl;
	std::string segmented_dirt_path;
	pnh.param("segmented_dirt_path", segmented_dirt_path, std::string("dirt_segmented"));
	std::cout << "segmented_dirt_path: " << segmented_dirt_path << std::endl;
	std::string segmented_dirt_mask_path;
	pnh.param("segmented_dirt_mask_path", segmented_dirt_mask_path, std::string("dirt_segmented"));
	std::cout << "segmented_dirt_mask_path: " << segmented_dirt_mask_path << std::endl;
	std::string segmented_objects_path;
	pnh.param("segmented_objects_path", segmented_objects_path, std::string("objects_segmented"));
	std::cout << "segmented_objects_path: " << segmented_objects_path << std::endl;
	std::string segmented_objects_mask_path;
	pnh.param("segmented_objects_mask_path", segmented_objects_mask_path, std::string("objects_segmented"));
	std::cout << "segmented_objects_mask_path: " << segmented_objects_mask_path << std::endl;
	std::string brightness_shadow_mask_path;
	pnh.param("brightness_shadow_mask_path", brightness_shadow_mask_path, std::string("brightness_shadow_masks"));
	std::cout << "brightness_shadow_mask_path: " << brightness_shadow_mask_path << std::endl;
	std::string illumination_mask_path;
	pnh.param("illumination_mask_path", illumination_mask_path, std::string("illumination_masks"));
	std::cout << "illumination_mask_path: " << illumination_mask_path << std::endl;
	std::string blended_ground_image_path;
	pnh.param("blended_ground_image_path", blended_ground_image_path, std::string("blended_floor_images"));
	std::cout << "blended_ground_image_path: " << blended_ground_image_path << std::endl;
	std::string blended_ground_image_mask_path;
	pnh.param("blended_ground_image_mask_path", blended_ground_image_mask_path, std::string("blended_floor_masks"));
	std::cout << "blended_ground_image_mask_path: " << blended_ground_image_mask_path << std::endl;
	std::string blended_ground_image_bbox_filename;
	pnh.param("blended_ground_image_bbox_filename", blended_ground_image_bbox_filename, std::string("bbox_labels.txt"));
	std::cout << "blended_ground_image_bbox_filename: " << blended_ground_image_bbox_filename << std::endl;
	int max_num_dirts;
	pnh.param("max_num_dirts", max_num_dirts, 8);
	std::cout << "max_num_dirts: " << max_num_dirts << std::endl;
	int min_num_dirts;
	pnh.param("min_num_dirts", min_num_dirts, 5);
	std::cout << "min_num_dirts: " << min_num_dirts << std::endl;
	int max_num_objects;
	pnh.param("max_num_objects", max_num_objects, 6);
	std::cout << "max_num_objects: " << max_num_objects << std::endl;
	int min_num_objects;
	pnh.param("min_num_objects", min_num_objects, 2);
	std::cout << "min_num_objects: " << min_num_objects << std::endl;
	bool flip_clean_ground;
	pnh.param("flip_clean_ground", flip_clean_ground, true);
	std::cout << "flip_clean_ground: " << flip_clean_ground << std::endl;
	int ground_image_reuse_times;
	pnh.param("ground_image_reuse_times", ground_image_reuse_times, 2);
	std::cout << "ground_image_reuse_times: " << ground_image_reuse_times << std::endl;

	// run image blender
	ipa_dirt_detection_dataset_tools::ImageBlender image_blend(base_path + ground_image_path, base_path + segmented_dirt_path, base_path + segmented_dirt_mask_path,
			base_path + segmented_objects_path, base_path + segmented_objects_mask_path, base_path + brightness_shadow_mask_path, base_path + illumination_mask_path,
			base_path + blended_ground_image_path, base_path + blended_ground_image_mask_path, base_path + blended_ground_image_path + blended_ground_image_bbox_filename,
			max_num_dirts, min_num_dirts, max_num_objects, min_num_objects, flip_clean_ground, ground_image_reuse_times);
	image_blend.run();

	return 0;
}
