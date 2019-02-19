#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ipa_dirt_detection_dataset_tools/image_blender.h"

ipa_dirt_detection_dataset_tools::ImageBlender::ImageBlender(const std::string& clean_ground_path, const std::string& segmented_dirt_path,
		const std::string& segmented_dirt_mask_path, const std::string& segmented_objects_path, const std::string& segmented_objects_mask_path,
		const std::string& brightness_shadow_mask_path, const std::string& blended_img_folder, const std::string& blended_mask_folder,
		const std::string& blended_img_bbox_filename, const int max_num_dirt, const int min_num_dirt, const int max_num_objects, const int min_num_objects,
		const bool flip_clean_ground, const int ground_image_reuse_times)
{
	clean_ground_path_ = clean_ground_path;

	segmented_dirt_path_ = segmented_dirt_path;
	segmented_dirt_mask_path_ = segmented_dirt_mask_path;
	segmented_objects_path_ = segmented_objects_path;
	segmented_objects_mask_path_ = segmented_objects_mask_path;
	brightness_shadow_mask_path_ = brightness_shadow_mask_path;

	blended_img_folder_ = blended_img_folder;
	blended_mask_folder_ = blended_mask_folder;
	blended_img_bbox_filename_ = blended_img_bbox_filename;

	max_num_dirt_ = max_num_dirt;
	min_num_dirt_ = min_num_dirt;

	max_num_objects_ = max_num_objects;
	min_num_objects_ = min_num_objects;

	flip_clean_ground_ = flip_clean_ground;
	ground_image_reuse_times_ = ground_image_reuse_times;


	img_cols_ = 1280;		// todo:
	img_rows_ = 1024;

	if_resize_dirt_ = 0;
	resize_ratio_ = 1.0;

	srand((int) time(0));
}

ipa_dirt_detection_dataset_tools::ImageBlender::~ImageBlender()
{
}

void ipa_dirt_detection_dataset_tools::ImageBlender::run()
{
	// create lists of all image files (clean images, dirt images and masks, object images, illumination and shadow images)
	collectImageFiles();

	//-------------------------------------------------------------------------------------------------------------------------------------
	std::ofstream blended_img_bbox_labels;
	blended_img_bbox_labels.open(blended_img_bbox_filename_.c_str());

	// start to blend images, reuse the ground pattern with different dirts
	for (int m = 0; m < clean_ground_filenames_.size(); ++m)
	{
		clean_ground_pattern_ = cv::imread(clean_ground_filenames_[m], CV_LOAD_IMAGE_COLOR);

		// todo: ground_image_reuse_times_ , also put into filenames

		//-------------------------------------------------------------------------------------------------------
		// create the file name of the blended ground
		std::vector<std::string> strs, strs1;
		boost::split(strs, clean_ground_filenames_[m], boost::is_any_of("\t,/"));			// folders and filename
		boost::split(strs1, strs[strs.size()-1], boost::is_any_of("\t,."));		// remove the suffix of the image name
		const std::string ground_file_name = strs[strs.size()-2] + "_" + strs1[0];
		std::cout << "FILE NAME " << ground_file_name << std::endl;
		//-------------------------------------------------------------------------------------------------------

		getchar();

		int dirt_num = min_num_dirt_ + rand() % (max_num_dirt_ - min_num_dirt_);		// generate number of dirts in range max_num_dirt and min_num_dirt
		int objects_num = min_num_objects_ + rand() % (max_num_objects_ - min_num_objects_);		// generate number of objects in range max_num_objects and min_num_objects

		blendImage(dirt_num, blended_img_bbox_labels, ground_file_name);     // First add artificial dirt , then add the objects for classification
		blendImage(objects_num, blended_img_bbox_labels, ground_file_name, true);

		serie_num_ = 0;
		cv::imwrite(blended_mask_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_);
		cv::imwrite(blended_mask_folder_ + "/mask_vis_" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_ * 255);
		cv::imwrite(blended_img_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_img_);

		if (flip_clean_ground_ == 1)
		{
			serie_num_ = 1;
			cv::flip(clean_ground_pattern_, fliped_ground_img_, 0);
			clean_ground_pattern_ = fliped_ground_img_;
			dirt_num = rand() % (max_num_dirt_ - min_num_dirt_) + min_num_dirt_;
			blendImage(dirt_num, blended_img_bbox_labels, ground_file_name);
			blendImage(objects_num, blended_img_bbox_labels, ground_file_name, true);

			cv::imwrite(blended_mask_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_);
			cv::imwrite(blended_mask_folder_ + "/mask_vis_" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_ * 255);
			cv::imwrite(blended_img_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_img_);

			serie_num_ = 2;
			cv::flip(clean_ground_pattern_, fliped_ground_img_, 1);
			clean_ground_pattern_ = fliped_ground_img_;
			dirt_num = rand() % (max_num_dirt_ - min_num_dirt_) + min_num_dirt_;
			blendImage(dirt_num, blended_img_bbox_labels, ground_file_name);
			blendImage(objects_num, blended_img_bbox_labels, ground_file_name, true);

			cv::imwrite(blended_mask_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_);
			cv::imwrite(blended_mask_folder_ + "/mask_vis_" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_ * 255);
			cv::imwrite(blended_img_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_img_);

			serie_num_ = 3;
			cv::flip(clean_ground_pattern_, fliped_ground_img_, 0);
			clean_ground_pattern_ = fliped_ground_img_;
			dirt_num = rand() % (max_num_dirt_ - min_num_dirt_) + min_num_dirt_;
			blendImage(dirt_num, blended_img_bbox_labels, ground_file_name);
			blendImage(objects_num, blended_img_bbox_labels, ground_file_name, true);

			cv::imwrite(blended_mask_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_);
			cv::imwrite(blended_mask_folder_ + "/mask_vis_" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_mask_ * 255);
			cv::imwrite(blended_img_folder_ + "/" + ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) + ".png", blended_img_);
		}
		edge_smoothing(3);
		// if (m % 5 == 1)    // 20% add shadow
		//   shadow_and_illuminance(1);
		// if (m % 5 == 2)
		//   shadow_and_illuminance(0);
	}
	blended_img_bbox_labels.close();
}


void ipa_dirt_detection_dataset_tools::ImageBlender::collectImageFiles()
{
	boost::filesystem::directory_iterator end_itr;

	// to list the clean ground file names
	for (boost::filesystem::directory_iterator itr1(clean_ground_path_); itr1 != end_itr; ++itr1)
	{
		for (boost::filesystem::directory_iterator itr2(itr1->path()); itr2 != end_itr; ++itr2)
			clean_ground_filenames_.push_back(itr2->path().string());
	}
	num_clean_ground_images_ = clean_ground_filenames_.size();
	std::cout << "The number of clean ground images is " << num_clean_ground_images_ << std::endl;
	std::sort(clean_ground_filenames_.begin(), clean_ground_filenames_.end());
	for (std::vector<std::string>::iterator it = clean_ground_filenames_.begin(); it != clean_ground_filenames_.end(); ++it)
		std::cout << "   - " << *it << std::endl;

	// to list the segmented dirt file names
	for (boost::filesystem::directory_iterator itr2(segmented_dirt_path_); itr2 != end_itr; ++itr2)
		if (itr2->path().string().find("_mask.png") == std::string::npos)
			segmented_dirt_filenames_.push_back(itr2->path().string());
	num_segmented_dirt_images_ = segmented_dirt_filenames_.size();
	std::cout << "\nThe number of dirt samples is " << num_segmented_dirt_images_ << std::endl;
	std::sort(segmented_dirt_filenames_.begin(), segmented_dirt_filenames_.end());
	for (std::vector<std::string>::iterator it = segmented_dirt_filenames_.begin(); it != segmented_dirt_filenames_.end(); ++it)
		std::cout << "   - " << *it << std::endl;

	// to list the segmented dirt mask file names
	for (boost::filesystem::directory_iterator itr3(segmented_dirt_mask_path_); itr3 != end_itr; ++itr3)
		if (itr3->path().string().find("_mask.png") != std::string::npos)
			segmented_dirt_mask_filenames_.push_back(itr3->path().string());
	std::cout << "\nThe number of dirt masks is " << segmented_dirt_mask_filenames_.size() << std::endl;
	std::sort(segmented_dirt_mask_filenames_.begin(), segmented_dirt_mask_filenames_.end());
	for (std::vector<std::string>::iterator it = segmented_dirt_mask_filenames_.begin(); it != segmented_dirt_mask_filenames_.end(); ++it)
		std::cout << "   - " << *it << std::endl;

	// to list the objects file names
	for (boost::filesystem::directory_iterator itr4(segmented_objects_path_); itr4 != end_itr; ++itr4)
		if (itr4->path().string().find("_mask.png") == std::string::npos)
			segmented_objects_filenames_.push_back(itr4->path().string());
	num_object_images_ = segmented_objects_filenames_.size();
	std::cout << "\nThe number of object samples is " << num_object_images_ << std::endl;
	std::sort(segmented_objects_filenames_.begin(), segmented_objects_filenames_.end());
	for (std::vector<std::string>::iterator it = segmented_objects_filenames_.begin(); it != segmented_objects_filenames_.end(); ++it)
		std::cout << "   - " << *it << std::endl;

	// to list the object mask file names
	for (boost::filesystem::directory_iterator itr5(segmented_objects_mask_path_); itr5 != end_itr; ++itr5)
		if (itr5->path().string().find("_mask.png") != std::string::npos)
			segmented_objects_mask_filenames_.push_back(itr5->path().string());
	std::cout << "\nThe number of object masks is " << segmented_objects_mask_filenames_.size() << std::endl;
	std::sort(segmented_objects_mask_filenames_.begin(), segmented_objects_mask_filenames_.end());
	for (std::vector<std::string>::iterator it = segmented_objects_mask_filenames_.begin(); it != segmented_objects_mask_filenames_.end(); ++it)
		std::cout << "   - " << *it << std::endl;

	// to list the brightness and shadow masks
	for (boost::filesystem::directory_iterator itr6(brightness_shadow_mask_path_); itr6 != end_itr; ++itr6)
		brightness_shadow_mask_filenames_.push_back(itr6->path().string());
	std::cout << "\nThe number of illumination and shadow masks is " << brightness_shadow_mask_filenames_.size() << std::endl;
	std::sort(brightness_shadow_mask_filenames_.begin(), brightness_shadow_mask_filenames_.end());
	for (std::vector<std::string>::iterator it = brightness_shadow_mask_filenames_.begin(); it != brightness_shadow_mask_filenames_.end(); ++it)
		std::cout << "   - " << *it << std::endl;
}


void ipa_dirt_detection_dataset_tools::ImageBlender::blendImage(int dirt_num, std::ofstream& myfile, const std::string& ground_file_name)
{
	int img_cols = clean_ground_pattern_.cols;
	int img_rows = clean_ground_pattern_.rows;

	blended_img_ = clean_ground_pattern_.clone();
	blended_mask_ = cv::Mat::zeros(img_rows, img_cols, CV_32S);

	for (int n = 0; n < dirt_num; n++)
	{
		int anchor_col = rand() % img_cols + 2;
		int anchor_row = rand() % img_rows + 2;     // top left point of the blending position

		if_resize_dirt_ = bool(rand() % 4);                // 20% of the dirt will be resized

		int dirt_image_index = rand() % num_segmented_dirt_images_ - 1;     // TODO for testing
		if (dirt_image_index == -1)
			dirt_image_index = 0;
		artificial_dirt_ = cv::imread(segmented_dirt_filenames_[dirt_image_index], CV_LOAD_IMAGE_COLOR);
		std::cout << segmented_dirt_filenames_[dirt_image_index] << std::endl;
		artificial_dirt_mask_ = cv::imread(segmented_dirt_mask_filenames_[dirt_image_index], CV_LOAD_IMAGE_GRAYSCALE);
		std::cout << segmented_dirt_mask_filenames_[dirt_image_index] << std::endl;

		// remove some edges TODO
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
		cv::morphologyEx(artificial_dirt_mask_, artificial_dirt_mask_, cv::MORPH_ERODE, element);

		if (if_resize_dirt_)
		{
			resize_dirt();
		}
		rotateImage();
		shrank_bounding_box();

		rotated_shranked_artificial_dirt_mask_.convertTo(rotated_shranked_artificial_dirt_mask_, CV_32S);
		rotated_schranked_artificial_dirt_.convertTo(rotated_schranked_artificial_dirt_, blended_img_.type());
		std::cout << "rotate matrix finished" << std::endl;
		int dirt_cols = rotated_schranked_artificial_dirt_.cols;
		int dirt_rows = rotated_schranked_artificial_dirt_.rows;

		if ((anchor_col + dirt_cols + 2) > img_cols)
			anchor_col = anchor_col - (dirt_cols + anchor_col + 2 - img_cols);    // add 2 free space
		if ((anchor_row + dirt_rows + 2) > img_rows)
			anchor_row = anchor_row - (dirt_rows + anchor_row + 2 - img_rows);

		// write the arguments of the bounding box in file
		myfile << ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) << " " << anchor_col - 2 << " " << anchor_row - 2 << " " << anchor_col + dirt_cols + 2 << " "
				<< anchor_row + dirt_rows + 2 << " " << "Dirt \n";

		for (int i = anchor_row; i < anchor_row + dirt_rows; i++)      // to blend the images
		{
			for (int j = anchor_col; j < anchor_col + dirt_cols; j++)
			{
				int mask_row = i - anchor_row;
				int mask_col = j - anchor_col;
				if (rotated_shranked_artificial_dirt_mask_.at<int>(mask_row, mask_col) > 0)        // data type of the mask !!!
				{
					blended_img_.at<cv::Vec3b>(i, j) = rotated_schranked_artificial_dirt_.at<cv::Vec3b>(mask_row, mask_col);
					blended_mask_.at<int>(i, j) = 1;
				}
			}
		}
		std::cout << "one dirt finished" << std::endl;
	}
	//blended_mask_.convertTo(blended_mask_, CV_8UC1);
	//std::cout << "datat type is" << artificial_dirt_mask_.type() << std::endl;

	//cv::imshow("test", blended_img_);
	//cv::waitKey(0);
}

void ipa_dirt_detection_dataset_tools::ImageBlender::blendImage(int dirt_num, std::ofstream& myfile, const std::string& ground_file_name, bool if_for_classification)
{
	int img_cols = clean_ground_pattern_.cols;
	int img_rows = clean_ground_pattern_.rows;

	//blended_img_ = clean_ground_pattern_.clone();
	//blended_mask_ = cv::Mat::zeros(img_rows, img_cols, CV_32S);

	for (int n = 0; n < dirt_num; n++)
	{
		int anchor_col = rand() % img_cols;
		int anchor_row = rand() % img_rows;     // top left point of the blending position

		int pens_image_index = rand() % num_object_images_ - 1;     // TODO for testing
		if (pens_image_index == -1)
			pens_image_index = 0;
		artificial_dirt_ = cv::imread(segmented_objects_filenames_[pens_image_index], CV_LOAD_IMAGE_COLOR);
		std::cout << segmented_objects_filenames_[pens_image_index] << std::endl;
		artificial_dirt_mask_ = cv::imread(segmented_objects_mask_filenames_[pens_image_index], CV_LOAD_IMAGE_GRAYSCALE);
		// remove some edges TODO
		//cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(3 , 3), cv::Point(1, 1) );
		//cv::morphologyEx(artificial_dirt_mask_, artificial_dirt_mask_, cv::MORPH_ERODE, element);

		std::cout << segmented_objects_mask_filenames_[pens_image_index] << std::endl;
		rotateImage();
		shrank_bounding_box();

		rotated_shranked_artificial_dirt_mask_.convertTo(rotated_shranked_artificial_dirt_mask_, CV_32S);
		rotated_schranked_artificial_dirt_.convertTo(rotated_schranked_artificial_dirt_, blended_img_.type());
		std::cout << "rotate matrix finished" << std::endl;
		int dirt_cols = rotated_schranked_artificial_dirt_.cols;
		int dirt_rows = rotated_schranked_artificial_dirt_.rows;

		if ((anchor_col + dirt_cols) > img_cols)
			break;   //anchor_col = anchor_col - (dirt_cols + anchor_col - img_cols);
		if ((anchor_row + dirt_rows) > img_rows)
			break;  //anchor_row = anchor_row - (dirt_rows + anchor_row - img_rows);

		std::string class_name;
		get_patch_classname(segmented_objects_filenames_[pens_image_index], class_name);

		// write the arguments of the bounding box in file
		myfile << ground_file_name + "_s" + ipa_dirt_detection_dataset_tools::to_string(serie_num_) << " " << anchor_col << " " << anchor_row << " " << anchor_col + dirt_cols << " "
				<< anchor_row + dirt_rows << " " << class_name << '\n';

		for (int i = anchor_row; i < anchor_row + dirt_rows; i++)      // to blend the images
		{
			for (int j = anchor_col; j < anchor_col + dirt_cols; j++)
			{
				int mask_row = i - anchor_row;
				int mask_col = j - anchor_col;
				if (rotated_shranked_artificial_dirt_mask_.at<int>(mask_row, mask_col) > 0)        // data type of the mask !!!
				{
					blended_img_.at<cv::Vec3b>(i, j) = rotated_schranked_artificial_dirt_.at<cv::Vec3b>(mask_row, mask_col);
					blended_mask_.at<int>(i, j) = 1;
				}
			}
		}
		std::cout << "one dirt finished" << std::endl;
	}
	blended_mask_.convertTo(blended_mask_, CV_8UC1);
}

// reference: https://www.pyimagesearch.com/2017/01/02/rotate-images-correctly-with-opencv-and-python/
void ipa_dirt_detection_dataset_tools::ImageBlender::rotateImage()
{
	//srand((int)time(0));               // generate random rotation angle
	double rotate_angle = rand() % 180;
	std::cout << "The rotation angle is " << rotate_angle << std::endl;

	int img_cols = artificial_dirt_.cols;
	int img_rows = artificial_dirt_.rows;

	cv::Point2f rotate_center = cv::Point(ceil(img_cols / 2), ceil(img_rows / 2));
	cv::Mat rotate_matrix = cv::getRotationMatrix2D(rotate_center, rotate_angle, 1.0);
	std::cout << rotate_matrix << std::endl;
	cv::Mat t = rotate_matrix.clone();
	std::cout << rotate_matrix.at<double>(0, 0) << rotate_matrix.at<double>(0, 1) << std::endl;

	float c = abs(rotate_matrix.at<double>(0, 0));
	float s = abs(rotate_matrix.at<double>(0, 1));

	int nW = img_rows * s + img_cols * c;
	int nH = img_rows * c + img_cols * s;

	rotate_matrix.at<double>(0, 2) += (nW / 2) - (img_cols / 2);
	rotate_matrix.at<double>(1, 2) += (nH / 2) - (img_rows / 2);

	std::cout << "start to rotate matrix" << std::endl;
	std::cout << img_cols << ' ' << img_rows << ' ' << nW << ' ' << nH << std::endl;
	rotated_artificial_dirt_.create(nH, nW, artificial_dirt_.type());
	rotated_artificial_dirt_mask_.create(nH, nW, artificial_dirt_mask_.type());

	if (artificial_dirt_.type() != CV_8UC3)
	{
		std::cout << "ImageFlip::imageCallback: Error: The image format of the color image is not CV_8UC3.\n" << std::endl;
	}

	std::cout << "rotating matrix" << std::endl;
	cv::warpAffine(artificial_dirt_, rotated_artificial_dirt_, rotate_matrix, rotated_artificial_dirt_.size());
	cv::warpAffine(artificial_dirt_mask_, rotated_artificial_dirt_mask_, rotate_matrix, rotated_artificial_dirt_mask_.size());

	//shrank_bounding_box();
}

void ipa_dirt_detection_dataset_tools::ImageBlender::shrank_bounding_box()
{
	int rows = rotated_artificial_dirt_.rows;
	int cols = rotated_artificial_dirt_.cols;

	int left_edge_flag = 0;
	int upper_edge_flag = 0;

	int left_edge = 0;
	int right_edge = 0;
	int upper_edge = 0;
	int down_edge = 0;
	cv::Mat frame_mask = rotated_artificial_dirt_mask_.clone();
	frame_mask.convertTo(frame_mask, CV_32S);

	// std::cout  << frame_mask << std::endl;

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (int(frame_mask.at<int>(i, j)) > 0 && left_edge_flag == 0)
			{
				std::cout << frame_mask.at<int>(i, j) << std::endl;
				left_edge = j;
				left_edge_flag = 1;
			}
			else if (int(frame_mask.at<int>(i, j)) > 0 && left_edge_flag == 1)
			{
				if (j > left_edge)
					right_edge = j > right_edge ? j : right_edge;
				else
					left_edge = j;
			}
			if (int(frame_mask.at<int>(i, j)) > 0 && upper_edge_flag == 0)
			{
				upper_edge = i;
				upper_edge_flag = 1;
			}
			else if (int(frame_mask.at<int>(i, j)) > 0 && upper_edge_flag == 1)
			{
				if (i > upper_edge)
					down_edge = i > down_edge ? i : down_edge;
				else
					upper_edge = i;
			}
		}
	}

	std::cout << "x, y, w, h: " << left_edge << " " << upper_edge << " " << right_edge - left_edge << " " << down_edge << " " << upper_edge << std::endl;
	cv::Rect roi;
	roi.x = left_edge;     // add some free space , specially for small objects in training
	if (left_edge >= 1)
		roi.x = left_edge;
	roi.y = upper_edge;
	if (upper_edge >= 1)
		roi.y = upper_edge;
	roi.width = right_edge - left_edge;
	//if ((right_edge - left_edge) >= cols + 1)
	//    roi.width = right_edge - left_edge + 1;
	roi.height = down_edge - upper_edge;
	//if ((down_edge - upper_edge) >= rows + 1)
	//    roi.height = down_edge - upper_edge + 1;

	rotated_schranked_artificial_dirt_ = rotated_artificial_dirt_(roi);
	rotated_shranked_artificial_dirt_mask_ = rotated_artificial_dirt_mask_(roi);

	// For TEST
	cv::imwrite("/home/robot/artificial dirt1.jpg", artificial_dirt_);
	cv::imwrite("/home/robot/rotate dirt1.jpg", rotated_artificial_dirt_);
	cv::imwrite("/home/robot/schrank dirt1.jpg", rotated_schranked_artificial_dirt_);

	//cv::imshow("shranked dirt", artificial_dirt_);
	//cv::waitKey(0);
}

// The function is used for extract the class name from the file name of the segmented patches
void ipa_dirt_detection_dataset_tools::ImageBlender::get_patch_classname(std::string& patch_name, std::string& class_name)
{
	std::vector<std::string> strs;
	boost::split(strs, patch_name, boost::is_any_of("\t,/"));
	std::string image_name = strs[strs.size() - 1];

	std::vector<std::string> splits;
	boost::split(splits, image_name, boost::is_any_of("\t,_"));
	class_name = splits[0];
	std::cout << "Class of the object is: " << class_name << std::endl;
}

void ipa_dirt_detection_dataset_tools::ImageBlender::edge_smoothing(int half_kernel_size)
{
	blended_img_.convertTo(blended_img_, CV_32F);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	// each contour is stored as a vector of points, than we only need to loop over these points to smooth the object edge
	cv::findContours(blended_mask_, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	/*
	 cv::Mat drawing = blended_img_.clone();
	 for( int i = 0; i< contours.size(); i++ )
	 {
	 cv::Scalar color = cv::Scalar(255,0,0);
	 cv::drawContours( drawing , contours, i, color, 2, 8, hierarchy, 0, Point() );
	 }
	 drawing = drawing / 255;
	 cv::imshow("drawing", drawing);
	 cv::waitKey(0);
	 */

	// gaussian blur kernel
	cv::Mat gaussian_x = cv::getGaussianKernel(half_kernel_size * 2 + 1, 0.5, CV_32F);   // second parameter is sigma
	cv::Mat gaussian_y = cv::getGaussianKernel(half_kernel_size * 2 + 1, 0.5, CV_32F);
	cv::Mat gaussian_kernel = gaussian_x * gaussian_y.t();
	std::cout << "Gaussian kernel is: " << gaussian_kernel << std::endl;

	// loop over the contour points for edge_smoothing
	int num_contours = contours.size();
	for (int contour_iter = 0; contour_iter < num_contours; contour_iter++)
	{
		std::vector<cv::Point> current_contour = contours[contour_iter];
		int num_points = current_contour.size();

		for (int point_iter = 0; point_iter < num_points; point_iter++)
		{
			cv::Point anchor_point = current_contour[point_iter];
			int point_x = anchor_point.x;
			int point_y = anchor_point.y;

			if ((point_x - half_kernel_size) < 0 || (point_x + half_kernel_size) >= img_cols_ || (point_y - half_kernel_size) < 0 || (point_y + half_kernel_size) >= img_rows_) // if the ROI is out of the image range, continue
				continue;

			cv::Mat image_roi;
			cv::Rect region_of_interest(point_x - half_kernel_size, point_y - half_kernel_size, 2 * half_kernel_size + 1, 2 * half_kernel_size + 1);
			image_roi = blended_img_(region_of_interest);

			// split three color channels
			std::vector<cv::Mat> rgb;
			cv::split(image_roi, rgb);
			cv::Mat b = rgb[0];
			float new_pixel_value_b = cv::sum(b.mul(gaussian_kernel))[0];
			cv::Mat g = rgb[1];
			float new_pixel_value_g = cv::sum(g.mul(gaussian_kernel))[0];
			cv::Mat r = rgb[2];
			float new_pixel_value_r = cv::sum(r.mul(gaussian_kernel))[0];

			blended_img_.at<cv::Vec3f>(point_y, point_x)[0] = new_pixel_value_b;      // check point_x and point_y order TODO
			blended_img_.at<cv::Vec3f>(point_y, point_x)[1] = new_pixel_value_g;
			blended_img_.at<cv::Vec3f>(point_y, point_x)[2] = new_pixel_value_r;

		}
	}
}

// The function used to add artificial shadows and brightness in synthetic images
void ipa_dirt_detection_dataset_tools::ImageBlender::shadow_and_illuminance(bool shadow_or_illuminance)
{
	// random generator for the shape and size
	int polygon_or_ellipse = rand() % 2;           // 1 for polygon and 0 for ellipse
	cv::Mat mask = cv::Mat::zeros(img_rows_, img_cols_, CV_32F);
	float f = (rand() % 5) / 10;           //  opacity factor as in Gimp

	if (polygon_or_ellipse)                        // assume the polygon with 4 points
	{
		cv::Point rook_points[1][4];
		int start_point_x = rand() % img_cols_;
		int start_point_y = rand() % img_rows_;
		float x1 = rand() % abs(start_point_x - 80) + 40;
		float y1 = rand() % abs(start_point_y - 80) + 40;
		float x2 = rand() % abs(start_point_x - 80) + 40;
		float y2 = rand() % abs(abs(start_point_y - img_rows_) - 80) + 40;
		float x3 = rand() % abs(abs(start_point_x - img_cols_) - 80) + 40;
		float y3 = rand() % abs(abs(start_point_y - img_rows_) - 80) + 40;
		float x4 = rand() % abs(abs(start_point_x - img_cols_) - 80) + 40;
		float y4 = rand() % abs(start_point_y - 80) + 40;
		// minimum coordinate distance between each point with the center is 80, maximun is 40

		rook_points[0][0] = cv::Point(x1, y1);          // start points
		rook_points[0][1] = cv::Point(x2, y2);
		rook_points[0][2] = cv::Point(x3, y3);
		rook_points[0][3] = cv::Point(x4, y4);

		const cv::Point* ppt[1] =
		{ rook_points[0] };
		int npt[] =
		{ 4 };

		cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255, 255, 255), 8);
	}
	else
	{
		int start_x = rand() % img_cols_;
		int start_y = rand() % img_rows_;        // the minimum length of the axis is 40, maximun is 80
		int axes_x = rand() % 80 + 60;
		int axes_y = rand() % 80 + 60;              // TODO check if out of image range cause any problems

		int angle = rand() % 360;
		int start_angle = rand() % 90;
		int end_angle = rand() % 180 + 180;

		cv::ellipse(mask, cv::Point(start_x, start_y), cv::Size(axes_x, axes_y), angle, start_angle, end_angle, cv::Scalar(255, 255, 255), -1, 8);
	}

	if (shadow_or_illuminance)      // 1 for shadow
	{
		mask.convertTo(mask, blended_img_.type());
		cv::GaussianBlur(mask, mask, cv::Size(31, 31), 50, 50);

		std::vector<cv::Mat> rgb;
		cv::split(blended_img_, rgb);
		rgb[0] = rgb[0] - f * mask;
		rgb[1] = rgb[1] - f * mask;
		rgb[2] = rgb[2] - f * mask;
		cv::merge(rgb, blended_img_);
	}
	else
	{
		mask.convertTo(mask, blended_img_.type());
		cv::GaussianBlur(mask, mask, cv::Size((img_cols_ / 3 / 2) * 2 + 1, (img_cols_ / 3 / 2) * 2 + 1), 50, 50);
		std::vector<cv::Mat> rgb;

		cv::split(blended_img_, rgb);
		rgb[0] = rgb[0] + f * mask;
		rgb[1] = rgb[1] + f * mask;
		rgb[2] = rgb[2] + f * mask;
		cv::merge(rgb, blended_img_);
	}
}

void ipa_dirt_detection_dataset_tools::ImageBlender::resize_dirt()
{
	std::cout << artificial_dirt_.cols << ' ' << artificial_dirt_.rows << std::endl;
	std::cout << artificial_dirt_mask_.cols << ' ' << artificial_dirt_mask_.rows << std::endl;
	resize_ratio_ = (rand() % 4 + 8) / 10.0;        // resize ratio in range 0.8 to 1.2
	std::cout << "resize ratio: " << resize_ratio_ << std::endl;

	cv::resize(artificial_dirt_, artificial_dirt_, cv::Size(0, 0), resize_ratio_, resize_ratio_, CV_INTER_NN);
	cv::resize(artificial_dirt_mask_, artificial_dirt_mask_, cv::Size(0, 0), resize_ratio_, resize_ratio_, CV_INTER_NN);
}

void ipa_dirt_detection_dataset_tools::ImageBlender::shadow_and_illuminance_new(bool shadow_or_illuminance)
{
	int num_masks = brightness_shadow_mask_filenames_.size();
	int mask_index = rand() % num_masks;
	std::string mask_filename = brightness_shadow_mask_filenames_[mask_index];
	cv::Mat mask = cv::imread(mask_filename, CV_LOAD_IMAGE_GRAYSCALE);

	float f = (rand() % 5) / 10;           //  opacity factor as in Gimp

	if (shadow_or_illuminance)      // 1 for shadow
	{
		mask.convertTo(mask, blended_img_.type());
		cv::GaussianBlur(mask, mask, cv::Size(31, 31), 50, 50);

		std::vector<cv::Mat> rgb;
		cv::split(blended_img_, rgb);
		rgb[0] = rgb[0] - f * mask;
		rgb[1] = rgb[1] - f * mask;
		rgb[2] = rgb[2] - f * mask;
		cv::merge(rgb, blended_img_);
	}
	else
	{
		mask.convertTo(mask, blended_img_.type());
		cv::GaussianBlur(mask, mask, cv::Size((img_cols_ / 3 / 2) * 2 + 1, (img_cols_ / 3 / 2) * 2 + 1), 50, 50);
		std::vector<cv::Mat> rgb;

		cv::split(blended_img_, rgb);
		rgb[0] = rgb[0] + f * mask;
		rgb[1] = rgb[1] + f * mask;
		rgb[2] = rgb[2] + f * mask;
		cv::merge(rgb, blended_img_);
	}
}
