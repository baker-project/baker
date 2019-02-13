#include <ros/ros.h>
#include <string>
#include "ipa_dirt_detection_dataset_tools/image_blender.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_blender");
	ros::NodeHandle pnh("~");

	// read parameters
	std::string ground_image_path;
	pnh.param("ground_image_path", ground_image_path, std::string("/home/rmb-jx/dataset_new/ground_image"));
	std::cout << "ground_image_path: " << ground_image_path << std::endl;
	std::string segmented_dirt_cropped_path;
	pnh.param("segmented_dirt_cropped_path", segmented_dirt_cropped_path, std::string("/home/rmb-jx/dataset_new/Dirt"));
	std::cout << "Path to save the segmented dirt after cropping: " << segmented_dirt_cropped_path << std::endl;
	std::string segmented_dirt_cropped_mask_path;
	pnh.param("segmented_dirt_cropped_mask_path", segmented_dirt_cropped_mask_path, std::string("/home/rmb-jx/dataset_new/Dirtmask"));
	std::cout << "segmented_dirt_cropped_mask_path: " << segmented_dirt_cropped_mask_path << std::endl;
	std::string segmented_objects_path;
	pnh.param("segmented_objects_path", segmented_objects_path, std::string("/home/rmb-jx/dataset_new/Object"));
	std::cout << "segmented_objects_path: " << segmented_objects_path << std::endl;
	std::string segmented_objects_mask_path;
	pnh.param("segmented_objects_mask_path", segmented_objects_mask_path, std::string("/home/rmb-jx/dataset_new/Objectmask"));
	std::cout << "segmented_objects_mask_path: " << segmented_objects_mask_path << std::endl;
	std::string blended_ground_image_path;
	pnh.param("blended_ground_image_path", blended_ground_image_path, std::string("/home/rmb-jx/dataset_new/blended_ground_image"));
	std::cout << "blended_ground_image_path: " << blended_ground_image_path << std::endl;
	std::string blended_ground_image_mask_path;
	pnh.param("blended_ground_image_mask_path", blended_ground_image_mask_path, std::string("/home/rmb-jx/dataset_new/blended_ground_mask"));
	std::cout << "blended_ground_image_mask_path: " << blended_ground_image_mask_path << std::endl;
	int max_num_dirts;
	pnh.param("max_num_dirts", max_num_dirts, 8);
	std::cout << "Maximum number of dirts per frame: " << max_num_dirts << std::endl;
	int min_num_dirts;
	pnh.param("min_num_dirts", min_num_dirts, 5);
	std::cout << "Minimum number of dirts per frame: : " << min_num_dirts << std::endl;
	int max_num_objects;
	pnh.param("max_num_objects", max_num_objects, 6);
	std::cout << "Maximum number of pens per frame: " << max_num_objects << std::endl;
	int min_num_objects;
	pnh.param("min_num_objects", min_num_objects, 2);
	std::cout << "Minimum number of pens per frame: : " << min_num_objects << std::endl;
	std::string filename_bbox_parameters;
	pnh.param("bbox_parameters_file", filename_bbox_parameters, std::string("/home/robot/dataset_new/bboxArgus.txt"));
	std::cout << "bbox_parameters_file: " << filename_bbox_parameters << std::endl;
	bool flip_clean_ground;
	pnh.param("flip_clean_ground", flip_clean_ground, true);
	std::cout << "flip_clean_ground: " << flip_clean_ground << std::endl;
	std::string brightness_shadow_mask_path;
	pnh.param("brightness_shadow_mask_path", brightness_shadow_mask_path, std::string("/home/rmb-jx/dataset/brightness_shadow_mask"));
	std::cout << "brightness_shadow_mask_path: " << brightness_shadow_mask_path << std::endl;

	// run image blender
	ipa_dirt_detection_dataset_tools::ImageBlend image_blend(ground_image_path, segmented_dirt_cropped_path, segmented_dirt_cropped_mask_path, segmented_objects_path,
			segmented_objects_mask_path, blended_ground_image_path, blended_ground_image_mask_path, max_num_dirts, min_num_dirts, max_num_objects, min_num_objects,
			filename_bbox_parameters, flip_clean_ground, brightness_shadow_mask_path);
	image_blend.run();

	return 0;
}
