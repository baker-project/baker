#include "ipa_dirt_detection_dataset_tools/simple_segmentation.h"

ipa_dirt_detection_dataset_tools::SimpleSegmentation::SimpleSegmentation(const std::string dirt_image_path, const std::string cropped_image_path,
		const std::string cropped_mask_path, const double foreground_rectangle_canny1, const double foreground_rectangle_canny2, const double foreground_rectangle_min_area,
		const double foreground_rectangle_target_area, const double foreground_rectangle_shape_threshold, const int foreground_rectangle_additional_cropping,
		const int crop_residual)
{
	source_image_path_ = dirt_image_path;
	cropped_image_path_ = cropped_image_path;
	cropped_mask_path_ = cropped_mask_path;
	foreground_rectangle_canny1_ = foreground_rectangle_canny1;
	foreground_rectangle_canny2_ = foreground_rectangle_canny2;
	foreground_rectangle_min_area_ = foreground_rectangle_min_area;
	foreground_rectangle_target_area_ = foreground_rectangle_target_area;
	foreground_rectangle_shape_threshold_ = foreground_rectangle_shape_threshold;
	foreground_rectangle_additional_cropping_ = foreground_rectangle_additional_cropping;
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

	// process all images in the directory
	for (boost::filesystem::directory_iterator itr(dirt_images); itr != end_itr; ++itr)
	{
		const boost::filesystem::directory_entry entry = *itr;
		std::string path = entry.path().string();
		std::cout << "Currently processing dirt image: " << path << std::endl;

		// load image
		cv::Mat dirt_frame = cv::imread(path, CV_LOAD_IMAGE_COLOR);

		// find rectangular background
		cv::RotatedRect best_fit_rectangle = findRectangularBackground(dirt_frame);

		// crop original image to inner axis aligned rectangle of target rectangular background
		cv::Mat cropped_image;
		removeUncontrolledBackground(dirt_frame, cropped_image, best_fit_rectangle);

		// segment the object or dirt from the monochrome background
		cv::Mat mask_frame;
		segment(cropped_image, mask_frame);

		// crop the image further to contain the object or dirt only
		cv::Mat cropped_dirt_frame, cropped_mask_frame;
		crop(cropped_image, mask_frame, cropped_dirt_frame, cropped_mask_frame);

		// optional, for result visualization
		examine(cropped_image, mask_frame);

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
		cv::imwrite(dirt_path, cropped_dirt_frame);
		cv::imwrite(mask_path, cropped_mask_frame);
	}
}


cv::RotatedRect ipa_dirt_detection_dataset_tools::SimpleSegmentation::findRectangularBackground(const cv::Mat& image)
{
	// convert image to gray scale, find Canny edges, find largest contours
	cv::Mat image_temp, image_canny;
	cv::cvtColor(image, image_temp, CV_BGR2GRAY);
	cv::Canny(image_temp, image_canny, foreground_rectangle_canny1_, foreground_rectangle_canny2_, 5);
	cv::dilate(image_canny, image_canny, cv::Mat(), cv::Point(-1,-1), 5);	// close 5 pixel gaps in contours
	cv::erode(image_canny, image_canny, cv::Mat(), cv::Point(-1,-1), 5);
//	cv::imshow("canny", image_canny);
//	cv::waitKey();

	std::vector < std::vector<cv::Point> > contours;
	std::vector < cv::Vec4i > hierarchy;
	cv::findContours(image_canny, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
//	cv::Mat image_disp = image.clone();
	std::map<double, cv::RotatedRect> rectangular_contours;		// maps contour area (measured relative to image size) to the respective found rotated rectangle
	const double image_area_inv = 1./((double)image.rows*image.cols);
	for (int current_contour = 0; current_contour < contours.size(); current_contour++)
	{
		const double contour_area = cv::contourArea(contours[current_contour]);
		if (contour_area * image_area_inv > foreground_rectangle_min_area_)	// the contour should fill at least foreground_rectangle_min_area_% of the image
		{
			cv::RotatedRect rr = cv::minAreaRect(contours[current_contour]);
			if (contour_area/(rr.size.height*rr.size.width) > foreground_rectangle_shape_threshold_)		// ensures a rectangular shape
			{
				rectangular_contours[contour_area * image_area_inv] = rr;
//				cv::drawContours(image_disp, contours, current_contour, cv::Scalar(0,255,0), 2, 8, hierarchy, 2);
			}
		}
	}
//	cv::imshow("contours", image_disp);
//	cv::waitKey();

	// select the contour with best area fit to the target rectangle
	cv::RotatedRect best_fit_rectangle(cv::Point2f(image.cols/2, image.rows/2), cv::Size2f(image.cols, image.rows), 0);	// take whole image as default
	double best_fit_area_distance = fabs(best_fit_rectangle.size.width*best_fit_rectangle.size.height*image_area_inv - foreground_rectangle_target_area_);
	for (std::map<double, cv::RotatedRect>::iterator it=rectangular_contours.begin(); it!=rectangular_contours.end(); ++it)
	{
		double dist = fabs(it->first - foreground_rectangle_target_area_);
		if (dist < best_fit_area_distance)
		{
			best_fit_area_distance = dist;
			best_fit_rectangle = it->second;
		}
	}

	return best_fit_rectangle;
}


void ipa_dirt_detection_dataset_tools::SimpleSegmentation::removeUncontrolledBackground(const cv::Mat& src_image, cv::Mat& cropped_image, const cv::RotatedRect& foreground_rectangle)
{
	// norm angle between [0,90) deg, i.e. convert foreground_rectangle.angle to angle alpha, which is the turn angle from the negative y-axis (image coordinate system) to side s (see below)
	double alpha = foreground_rectangle.angle;
	while (alpha < 0)
		alpha += 90;
	while (alpha-90 >= 0)
		alpha -= 90;

	std::cout << foreground_rectangle.angle << ", " << alpha << std::endl;

	cv::Rect aligned_foreground_rectangle;
	if (alpha==0)
	{
		// the rotated rectangle is already axis-aligned
		aligned_foreground_rectangle = foreground_rectangle.boundingRect();
		aligned_foreground_rectangle.x += foreground_rectangle_additional_cropping_;
		aligned_foreground_rectangle.y += foreground_rectangle_additional_cropping_;
		aligned_foreground_rectangle.width -= 2*foreground_rectangle_additional_cropping_;
		aligned_foreground_rectangle.height -= 2*foreground_rectangle_additional_cropping_;
	}
	else
	{
		// take the largest axis aligned inner rectangle inside the foreground_rectangle
		// s = the rectangle side from the leftmost corner to the topmost corner
		// l = the rectangle side from the leftmost corner to the bottommost corner
		// alpha is the turn angle from the negative y-axis (image coordinate system) to side s
		// a = is a ratio of side s, a in [0,1]
		// there is a rectangular triangle with sides a*s and l1 (first part of l) around the 90deg angle
		// there is a rectangular triangle with sides (1-a)*s and l2 (second part of l) around the 90deg angle
		// ||l|| = ||l1||             + ||l2||
		// ||l|| = a*||s||*tan(alpha) + (1-a)*||s||/tan(alpha)
		// ==> a = ((||l||/||s||)*tan(alpha)-1) / (tan(alpha)*tan(alpha)-1)

		cv::Point2f corners[4];
		foreground_rectangle.points(corners);		// the corners are provide in the order: bottomLeft, topLeft, topRight, bottomRight

		// sort the points according to their x-coordinates from left to right and from top to bottom
		std::map<int,cv::Point2f> x_sorted_corners, y_sorted_corners;
		for (int i=0; i<4; ++i)
			x_sorted_corners[corners[i].x] = corners[i];
		for (int i=0; i<4; ++i)
			y_sorted_corners[corners[i].y] = corners[i];

		// find the left, right, top, and bottom points
		cv::Point2f& left = x_sorted_corners.begin()->second;
		cv::Point2f& right = (--x_sorted_corners.end())->second;
		cv::Point2f& top = y_sorted_corners.begin()->second;
		cv::Point2f& bottom = (--y_sorted_corners.end())->second;
		std::cout << "left=" << left << std::endl;
		std::cout << "right=" << right << std::endl;
		std::cout << "top=" << top << std::endl;
		std::cout << "bottom=" << bottom << std::endl;

		// compute a
		const double s = cv::norm(top-left);
		const double l = cv::norm(bottom-left);
		const double tan_alpha = tan(alpha*CV_PI/180.);
		const double a = (l/s * tan_alpha - 1) / (tan_alpha*tan_alpha - 1);
		std::cout << "s=" << s << ",  l=" << l << ",  alpha=" << alpha << ",  a=" << a << std::endl;

		// construct the rectangle
		const cv::Point2f upper_left = left + a*(top-left);
		const cv::Point2f lower_right = right + a*(bottom-right);
		aligned_foreground_rectangle.x = upper_left.x + foreground_rectangle_additional_cropping_;
		aligned_foreground_rectangle.y = upper_left.y + foreground_rectangle_additional_cropping_;
		aligned_foreground_rectangle.width = lower_right.x-upper_left.x - 2*foreground_rectangle_additional_cropping_;
		aligned_foreground_rectangle.height = lower_right.y-upper_left.y - 2*foreground_rectangle_additional_cropping_;
		std::cout << "aligned_foreground_rectangle=" << aligned_foreground_rectangle << std::endl;
	}

	cropped_image = cv::Mat(src_image, aligned_foreground_rectangle);

	cv::imshow("cropped", cropped_image);
	cv::waitKey();
}


void ipa_dirt_detection_dataset_tools::SimpleSegmentation::segment(const cv::Mat& image, cv::Mat& mask_frame)
{
	cv::Mat image_smoothed, image_preprocessed;
	cv::medianBlur(image, image_smoothed, 3);


	cv::Mat hsv;
	cv::cvtColor(image_smoothed, image_preprocessed, CV_BGR2Lab);
	std::vector<cv::Mat> hsv_vec;
	cv::split(image_preprocessed, hsv_vec);
	cv::imshow("hue", hsv_vec[0]);
	cv::imshow("sat", hsv_vec[1]);
	cv::imshow("val", hsv_vec[2]);
	cv::waitKey();


	const int color_segmentation_theshold = 40;		// todo: param

	// get average color
	const int avg_color_area_side_length = 10;
	cv::Scalar mean_colors(0);
	mean_colors += cv::mean(cv::Mat(image_preprocessed, cv::Rect(0, 0, avg_color_area_side_length, avg_color_area_side_length)));
	mean_colors += cv::mean(cv::Mat(image_preprocessed, cv::Rect(image_preprocessed.cols-avg_color_area_side_length-1, 0, avg_color_area_side_length, avg_color_area_side_length)));
	mean_colors += cv::mean(cv::Mat(image_preprocessed, cv::Rect(0, image_preprocessed.rows-avg_color_area_side_length-1, avg_color_area_side_length, avg_color_area_side_length)));
	mean_colors += cv::mean(cv::Mat(image_preprocessed, cv::Rect(image_preprocessed.cols-avg_color_area_side_length-1, image_preprocessed.rows-avg_color_area_side_length-1, avg_color_area_side_length, avg_color_area_side_length)));
	mean_colors *= 0.25;

	// create segmentation mask
	mask_frame = cv::Mat::zeros(image_preprocessed.rows, image_preprocessed.cols, CV_8UC1);
	for (int h = 0; h < image_preprocessed.rows; h++)
	{
		for (int w = 0; w < image_preprocessed.cols; w++)
		{
			const cv::Vec3b& intensity = image_preprocessed.at<cv::Vec3b>(h, w);
			if (fabs((double)intensity[0] - mean_colors[0]) > color_segmentation_theshold || fabs((double)intensity[1] - mean_colors[1]) > color_segmentation_theshold ||
					fabs((double)intensity[2] - mean_colors[2]) > color_segmentation_theshold)
			{
				mask_frame.at<uchar>(h, w) = 255;
			}
		}
	}

//	int morph_size = 1;                                             // Optional !!!!!!!!!!!!!!!!!
//	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1));
//	cv::morphologyEx(mask_frame, mask_frame, cv::MORPH_OPEN, element);
//	cv::morphologyEx(mask_frame, mask_frame, cv::MORPH_CLOSE, element);

	//cv::dilate(mask_frame , mask_frame, element);
}


void ipa_dirt_detection_dataset_tools::SimpleSegmentation::crop(const cv::Mat& dirt_frame, const cv::Mat& mask_frame, cv::Mat& cropped_dirt_frame, cv::Mat& cropped_mask_frame)
{
	int left_edge = mask_frame.cols-1;
	int right_edge = 0;
	int top_edge = mask_frame.rows-1;
	int bottom_edge = 0;

	bool left_edge_flag = 0;
	bool upper_edge_flag = 0;

	for (int v=0; v<mask_frame.rows; ++v)
	{
		for (int u=0; u<mask_frame.cols; ++u)
		{
			if (mask_frame.at<uchar>(v,u) == 255)
			{
				left_edge = std::min(left_edge, u);
				right_edge = std::max(right_edge, u);
				top_edge = std::min(top_edge, v);
				bottom_edge = std::max(bottom_edge, v);
			}
		}
	}

	std::cout << left_edge << ' ' << right_edge << std::endl;
	std::cout << top_edge << ' ' << bottom_edge << std::endl;

	const int max_x = std::min(dirt_frame.cols-1, right_edge+crop_residual_);
	const int max_y = std::min(dirt_frame.rows-1, bottom_edge+crop_residual_);
	cv::Rect roi;
	roi.x = std::max(0, left_edge-crop_residual_);
	roi.y = std::max(0, top_edge-crop_residual_);
	roi.width = max_x - roi.x;
	roi.height = max_y - roi.y;

	cropped_dirt_frame = dirt_frame(roi);
	cropped_mask_frame = mask_frame(roi);

//	int morph_size = 3;                                             // Optional !!!!!!!!!!!!!!!!!
//	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1));
//	cv::morphologyEx(cropped_mask_frame, cropped_mask_frame, cv::MORPH_CLOSE, element);

	cv::imshow("cropped_mask_frame", cropped_mask_frame);
	cv::waitKey(0);
}

void ipa_dirt_detection_dataset_tools::SimpleSegmentation::examine(const cv::Mat& dirt_frame, const cv::Mat& mask_frame)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat mask = mask_frame.clone();
	cv::findContours(mask, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	cv::Mat drawing = dirt_frame.clone();
	cv::drawContours(drawing, contours, -1, cv::Scalar(255, 0, 0), 2, 8, hierarchy);

	cv::imshow("drawing", drawing);
	cv::waitKey(0);
}
