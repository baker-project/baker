#include <ros/ros.h>
#include <string>
#include "ipa_dirt_detection_dataset_tools/simple_segmentation.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_segmentation");
	ros::NodeHandle pnh("~");

	// read parameters
	std::string dirt_image_path;
	pnh.param("images_to_be_segmented_path", dirt_image_path, std::string("/home/rmb-jx/dataset_new/Dirt"));
	std::cout << "images_to_be_segmented_path: " << dirt_image_path << std::endl;
	std::string cropped_image_path;
	pnh.param("segmented_dirt_cropped_path", cropped_image_path, std::string("/home/rmb-jx/dataset_new/Dirt"));
	std::cout << "Path to save the segmented dirt after cropping: " << cropped_image_path << std::endl;
	std::string cropped_mask_path;
	pnh.param("segmented_dirt_cropped_mask_path", cropped_mask_path, std::string("/home/rmb-jx/dataset_new/Dirtmask"));
	std::cout << "segmented_dirt_cropped_mask_path: " << cropped_mask_path << std::endl;
	int crop_residual;
	pnh.param("crop_residual", crop_residual, 2);
	std::cout << "Border residual for cropping bounding box is: " << crop_residual << std::endl;

	// run segmentation
	ipa_dirt_detection_dataset_tools::SimpleSegmentation segment_dirt(dirt_image_path, cropped_image_path, cropped_mask_path, crop_residual);
	segment_dirt.run();

	return 0;
}
