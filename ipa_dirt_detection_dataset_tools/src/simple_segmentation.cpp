#include "ipa_dirt_detection_dataset_tools/simple_segmentation.h"

ipa_dirt_detection_dataset_tools::SimpleSegmentation::SimpleSegmentation(const std::string dirt_image_path, const std::string cropped_image_path, const std::string cropped_mask_path,
		const int crop_residual)
{
	source_image_path_ = dirt_image_path;
	cropped_image_path_ = cropped_image_path;
	cropped_mask_path_ = cropped_mask_path;
	crop_residual_ = crop_residual;

	if (boost::filesystem::exists(source_image_path_) == true)
		std::cout << "There three paths are: " << source_image_path_ << std::endl << cropped_image_path_ << std::endl << cropped_mask_path_ << std::endl;
	else
		std::cout << "The source image path '" << source_image_path_ << "' does not exist." << std::endl;

	if (boost::filesystem::exists(cropped_image_path_) == false)
		boost::filesystem::create_directory(cropped_image_path_);
	if (boost::filesystem::exists(cropped_mask_path_) == false)
		boost::filesystem::create_directory(cropped_mask_path_);
}

ipa_dirt_detection_dataset_tools::SimpleSegmentation::~SimpleSegmentation()
{
}

void ipa_dirt_detection_dataset_tools::SimpleSegmentation::run()
{
	boost::filesystem::path dirt_images(source_image_path_);
	boost::filesystem::directory_iterator end_itr;

	for (boost::filesystem::directory_iterator itr(dirt_images); itr != end_itr; ++itr)
	{
		const boost::filesystem::directory_entry entry = *itr;
		std::string path = entry.path().string();
		std::cout << "Currently processing dirt image: " << path << std::endl;

		dirt_frame_ = cv::imread(path, CV_LOAD_IMAGE_COLOR);
		dirt_frame_.convertTo(dirt_frame_, CV_32F);
		segment(dirt_frame_);
		crop();

		examine();    // optional, for result visualization

		// split the image name from the data path
		std::vector<std::string> strs;
		std::string file_name;
		boost::split(strs, path, boost::is_any_of("\t,/"));
		for (std::vector<std::string>::iterator it = strs.begin(); it != strs.end(); it++)
		{
			std::cout << *it << std::endl;
			file_name = *it;
		}
		strs.clear();                // file_name, something like Pen000.png
		boost::split(strs, file_name, boost::is_any_of("\t,."));
		std::cout << "image name is: " << strs[0] << std::endl;

		std::string dirt_path = cropped_image_path_ + '/' + strs[0] + ".png";
		std::string mask_path = cropped_mask_path_ + '/' + strs[0] + "_mask.png";
		std::cout << dirt_path << std::endl;
		std::cout << mask_path << std::endl;
		cv::imwrite(dirt_path, cropped_dirt_frame_);
		cv::imwrite(mask_path, cropped_mask_frame_);
	}
}

void ipa_dirt_detection_dataset_tools::SimpleSegmentation::segment(const cv::Mat& image)
{

	int cols = image.cols;
	int rows = image.rows;

	float average_blue = 0;
	float average_green = 0;
	float average_red = 0;

	mask_frame_ = cv::Mat::zeros(rows, cols, CV_32F);

	for (int h = 0; h < 10; h++)
	{
		for (int w = 0; w < 10; w++)
		{
			cv::Vec3f intensity = image.at<cv::Vec3f>(h, w);
			float blue = intensity[0];
			float green = intensity[1];
			float red = intensity[2];

			average_blue += blue;
			average_green += green;
			average_red += red;
		}
	}

	average_blue /= 100;
	average_green /= 100;
	average_red /= 100;

	for (int h = 0; h < rows; h++)
	{
		for (int w = 0; w < cols; w++)
		{
			cv::Vec3f intensity = image.at<cv::Vec3f>(h, w);
			float blue = intensity[0];
			float green = intensity[1];
			float red = intensity[2];

			if (abs(green - average_green) > 50 || abs(blue - average_blue) > 50 || abs(red - average_red) > 50) // The threshold 15 here can be changed TODO  || abs(blue - average_blue) > 60 || abs(red - average_red) > 60
			{
				mask_frame_.at<float>(h, w) = 1;
			}
		}
	}
	int morph_size = 1;                                             // Optional !!!!!!!!!!!!!!!!!
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1));
	cv::morphologyEx(mask_frame_, mask_frame_, cv::MORPH_OPEN, element);
	cv::morphologyEx(mask_frame_, mask_frame_, cv::MORPH_CLOSE, element);

	//cv::dilate(mask_frame_ , mask_frame_, element);

	//std::cout << mask_frame_ << std::endl;
}

/*
 void ipa_dirt_detection_dataset_tools::SimpleSegment::segement_singlechannel(int threshold_range)
 {
 cv::Mat gray_input(dirt_frame_.size(), CV_8UC1);
 gray_input = cv::cvtColor(dirt_frame_, gray_input ,CV_RGB2GRAY);

 int average_threshold = 0;
 for(int h = 0; h < 10; h++)
 {
 for (int w = 0; w < 10; w++)
 {
 average_threshold = average_threshold + gray_input.at<int>(h, w);
 }
 }
 average_threshold = average_threshold / 100;

 cv::Mat mask = cv::Mat::zeros(gray_input.rows, gray_input.cols, CV_8UC1);
 for (int h = 0; h < gray_input.rows; h++)
 {
 for (int w = 0; w < gray_input.cols; w++)
 {
 if (abs(gray_input.at<int>(h, w)- average_threshold) > threshold_range)
 mask.at<int>(h, w) = 1;
 }
 }
 int morph_size = 1;                                             // Optional !!!!!!!!!!!!!!!!!
 cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ));
 cv::morphologyEx( mask, mask, cv::MORPH_OPEN, element );
 }
 */

void ipa_dirt_detection_dataset_tools::SimpleSegmentation::crop()
{
	int cols = mask_frame_.cols;
	int rows = mask_frame_.rows;

	int left_edge = 0;
	int right_edge = 0;
	int upper_edge = 0;
	int down_edge = 0;

	bool left_edge_flag = 0;
	bool upper_edge_flag = 0;

	cv::Mat mask_frame = mask_frame_.clone();       // only convert to 32S, it can be used for check if 0 or 1
	mask_frame.convertTo(mask_frame, CV_32S);

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			// -----------------------------------------------------------------------------
			if (mask_frame.at<int>(i, j) == 1 && left_edge_flag == 0)
			{
				left_edge = j;
				left_edge_flag = 1;
			}
			else if (mask_frame.at<int>(i, j) == 1 && left_edge_flag == 1)
			{
				if (j > left_edge)
					right_edge = j > right_edge ? j : right_edge;
				else
					left_edge = j;
			}
			if (mask_frame.at<int>(i, j) == 1 && upper_edge_flag == 0)
			{
				upper_edge = i;
				upper_edge_flag = 1;
			}
			else if (mask_frame.at<int>(i, j) == 1 && upper_edge_flag == 1)
			{
				if (i > upper_edge)
					down_edge = i > down_edge ? i : down_edge;
				else
					upper_edge = i;
			}
			// ----------------------------------------------------------------------------
		}
	}

	std::cout << left_edge << ' ' << right_edge << std::endl;
	std::cout << upper_edge << ' ' << down_edge << std::endl;

	cv::Mat dirt_frame_pad_array = dirt_frame_.clone();
	cv::Mat mask_frame_pad_array = mask_frame_.clone();

	cv::copyMakeBorder(dirt_frame_, dirt_frame_pad_array, crop_residual_, crop_residual_, crop_residual_, crop_residual_, cv::BORDER_REPLICATE);
	cv::copyMakeBorder(mask_frame_, mask_frame_pad_array, crop_residual_, crop_residual_, crop_residual_, crop_residual_, cv::BORDER_REPLICATE);

	cv::Rect roi;
	roi.x = left_edge;                                          //(left_edge - crop_residual) > 0 ? (left_edge - crop_residual) : 0;
	roi.y = upper_edge;                                         //(upper_edge - crop_residual) > 0 ? (upper_edge - crop_residual) : 0;
	roi.width = abs(right_edge - left_edge) + 2 * crop_residual_;
	roi.height = abs(down_edge - upper_edge) + 2 * crop_residual_;

	cropped_dirt_frame_ = dirt_frame_pad_array(roi);
	cropped_mask_frame_ = mask_frame_pad_array(roi);

	int morph_size = 3;                                             // Optional !!!!!!!!!!!!!!!!!
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1));
	cv::morphologyEx(cropped_mask_frame_, cropped_mask_frame_, cv::MORPH_CLOSE, element);

	cv::imshow("cropped_mask_frame", cropped_mask_frame_);
	cv::waitKey(0);
}

void ipa_dirt_detection_dataset_tools::SimpleSegmentation::examine()
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	std::cout << "mask_frame_ type: " << mask_frame_.type() << std::endl;
	cv::Mat mask = mask_frame_.clone();
	mask.convertTo(mask, CV_8UC1);
	cv::findContours(mask, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	cv::Mat drawing = dirt_frame_.clone();
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(255, 0, 0);
		cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
	}

	drawing = drawing / 255;
	cv::imshow("drawing", drawing);
	cv::waitKey(0);
}
