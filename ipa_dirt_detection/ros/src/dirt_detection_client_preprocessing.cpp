#include "ipa_dirt_detection/dirt_detection_client_preprocessing.h"


struct lessPoint2i: public std::binary_function<cv::Point2i, cv::Point2i, bool>
{
	bool operator()(const cv::Point2i& a, const cv::Point2i& b) const
	{
		return ((a.x < b.x) || ((a.x == b.x) && (a.y < b.y)));
	}
};

IpaDirtDetectionPreprocessing::ClientPreprocessing::ClientPreprocessing(ros::NodeHandle node_handle) :
				node_handle_(node_handle), dirt_detection_client_("/ipa_dirt_detection_action", true), it_(0)
{
	//PARAMS
	ros::NodeHandle pnh("~");
	std::cout << "\n========== dirt_detection_client_preprocessing Parameters ==========\n";

	pnh.param("use_mask", use_mask_, true);
	std::cout << "ClientPreprocessing: use_mask: " << use_mask_ << std::endl;
	pnh.param("warp_image", warp_image_, true);
	std::cout << "ClientPreprocessing: warp_image = " << warp_image_ << std::endl;
	pnh.param("floor_search_iterations", floor_search_iterations_, 3);
	std::cout << "ClientPreprocessing: floor_search_iterations = " << floor_search_iterations_ << std::endl;
	pnh.param("min_plane_points", min_plane_points_, 0);
	std::cout << "ClientPreprocessing: min_plane_points = " << min_plane_points_ << std::endl;
	pnh.param("max_distance_to_camera", max_distance_to_camera_, 300.0);
	std::cout << "ClientPreprocessing: max_distance_to_camera = " << max_distance_to_camera_ << std::endl;
	pnh.param("bird_eye_start_resolution", bird_eye_start_resolution_, 75.0);
	std::cout << "ClientPreprocessing: bird_eye_start_resolution = " << bird_eye_start_resolution_ << std::endl;
	pnh.param("bird_eye_base_resolution", bird_eye_base_resolution_, 300.0);
	std::cout << "ClientPreprocessing: bird_eye_base_resolution = " << bird_eye_base_resolution_ << std::endl;
	bird_eye_resolution_ = bird_eye_base_resolution_;
	pnh.param("detect_scales", detect_scales_, 5);
	std::cout << "ClientPreprocessing: detect_scales_ = " << detect_scales_ << std::endl;
	pnh.param("dirt_detection_activated_on_startup", dirt_detection_callback_active_, true);
	std::cout << "dirt_detection_activated_on_startup = " << dirt_detection_callback_active_ << std::endl;

	//DEBUG PARAMS
	pnh.param("show_original_image", debug_["show_original_image"], false);
	std::cout << "DEBUG_ClientPreprocessing: show_original_image = " << debug_["show_original_image"] << std::endl;
	pnh.param("save_data_for_test", debug_["save_data_for_test"], false);
	std::cout << "DEBUG_ClientPreprocessing: save_data_for_test = " << debug_["save_data_for_test"] << std::endl;
	pnh.param("show_plane_color_image", debug_["show_plane_color_image"], false);
	std::cout << "DEBUG_ClientPreprocessing: show_plane_color_image = " << debug_["show_plane_color_image"] << std::endl;

	// dynamic reconfigure
	dynamic_reconfigure::Server<ipa_dirt_detection::DirtDetectionPreprocessingConfig>::CallbackType dynamic_reconfigure_callback_type;
	dynamic_reconfigure_callback_type = boost::bind(&IpaDirtDetectionPreprocessing::ClientPreprocessing::dynamicReconfigureCallback, this, _1, _2);
	dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_type);

	// wait for server to start
	ROS_INFO("Starting client. Waiting for server of '/ipa_dirt_detection_action' to start.");
	dirt_detection_client_.waitForServer();

	//Subscribing point cloud from camera and find plane and compute bird's eye perspective.
	camera_depth_points_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 1, &IpaDirtDetectionPreprocessing::ClientPreprocessing::preprocessingCallback, this);

	//Publisher for plane segmented image.
	floor_plane_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("floor_plane", 1);

	//After action server did action we publish image containing the dirt positions.
	dirt_detected_ = node_handle_.advertise<cob_object_detection_msgs::DetectionArray>("dirt_detector_topic", 1);

	//todo: implement if needed
//	it_ = new image_transport::ImageTransport(node_handle_);
//	dirt_detection_image_pub_ = it_->advertise("dirt_detections", 1);

	// services
	activate_dirt_detection_service_server_ = node_handle_.advertiseService("activate_detection", &IpaDirtDetectionPreprocessing::ClientPreprocessing::activateDirtDetection, this);
	deactivate_dirt_detection_service_server_ = node_handle_.advertiseService("deactivate_detection", &IpaDirtDetectionPreprocessing::ClientPreprocessing::deactivateDirtDetection, this);

	std::cout << "Dirt detection preprocessing initialized." << std::endl;
}

IpaDirtDetectionPreprocessing::ClientPreprocessing::~ClientPreprocessing()
{
	if (it_ != 0)
		delete it_;
}

//###############################################################################################################
//======================== Function declaration =================================================================
//===============================================================================================================

bool IpaDirtDetectionPreprocessing::ClientPreprocessing::activateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ROS_INFO("Activating dirt detection.");
	dirt_detection_callback_active_ = true;

	res.success = true;

	return true;
}

bool IpaDirtDetectionPreprocessing::ClientPreprocessing::deactivateDirtDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ROS_INFO("Deactivating dirt detection.");
	dirt_detection_callback_active_ = false;

	res.success = true;

	return true;
}

void IpaDirtDetectionPreprocessing::ClientPreprocessing::dynamicReconfigureCallback(ipa_dirt_detection::DirtDetectionPreprocessingConfig &config, uint32_t level)
{
	//ROS_INFO("Reconfigure Request: %d %f %s %s %d",	config.int_param, config.double_param, config.str_param.c_str(), config.bool_param?"True":"False", config.size);
	warp_image_ = config.warp_image;
	bird_eye_base_resolution_ = config.bird_eye_base_resolution;
	max_distance_to_camera_ = config.max_distance_to_camera;
	floor_search_iterations_ = config.floor_search_iterations;
	min_plane_points_ = config.min_plane_points;

	std::cout << "\n========== dirt_detection_client_preprocessing Dynamic reconfigure ==========\n";
	std::cout << "  warp_image = " << warp_image_ << std::endl;
	std::cout << "  bird_eye_base_resolution = " << bird_eye_base_resolution_ << std::endl;
	std::cout << "  max_distance_to_camera = " << max_distance_to_camera_ << std::endl;
	std::cout << "  floor_search_iterations = " << floor_search_iterations_ << std::endl;
	std::cout << "  min_plane_points = " << min_plane_points_ << std::endl;
}

void IpaDirtDetectionPreprocessing::ClientPreprocessing::preprocessingCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{
	if (dirt_detection_callback_active_ == false)
		return;

	// get tf between camera and map
	tf::StampedTransform transformMapCamera;	// todo: add parameter for preferred relative transform between camera z-axis and ground plane z-axis and check this in planeSegmentation()
	transformMapCamera.setIdentity();

	// convert point cloud message
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*point_cloud2_rgb_msg, *input_cloud); //conversion Ros message->Pcl point cloud
	//std::cout << input_cloud->size() << std::endl;

//	Timer tim;
//	double segmentation_time = 0., dirt_detection_time = 0.;

	// find ground plane
	cv::Mat plane_color_image = cv::Mat();
	cv::Mat plane_mask = cv::Mat();
	pcl::ModelCoefficients plane_model;
	bool found_plane = planeSegmentation(input_cloud, point_cloud2_rgb_msg->header, plane_color_image, plane_mask, plane_model, transformMapCamera);

	// Paramter to use mask or not. If not all points belong to plane.
	//if(use_mask_ == false)
	//	plane_mask.setTo(cv::Scalar(255));

//	std::cout << "Segmentation time: " << tim.getElapsedTimeInMilliSec() << "ms." << std::endl;
//	segmentation_time = tim.getElapsedTimeInMilliSec();
//	tim.start();

	for (int s = 0; s < detect_scales_; s++)
	{
		// check if a ground plane could be found
		if (found_plane == true)
		{
			if (detect_scales_==1)
			{
				// single scale
				image_scaling_ = 1;
				bird_eye_resolution_ = bird_eye_base_resolution_;
			}
			else
			{
				image_scaling_ = pow(2, s) * bird_eye_start_resolution_ / bird_eye_base_resolution_;
				bird_eye_resolution_ = pow(2, s) * bird_eye_start_resolution_;
			}

			// remove perspective from image
			cv::Mat H; // homography between floor plane in image and bird's eye perspective
			cv::Mat R, t; // transformation between world and floor plane coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
			cv::Point2f camera_image_plane_offset; // offset in the camera image plane. Conversion from floor plane to  [xc, yc]
			cv::Mat plane_color_image_warped;
			cv::Mat plane_mask_warped;

			if (warp_image_ == true)
			{
				bool transformSuccessful = computeBirdsEyePerspective(input_cloud, plane_color_image, plane_mask, plane_model, H, R, t, camera_image_plane_offset, plane_color_image_warped, plane_mask_warped);
				if (transformSuccessful == false)
				{
					std::cout << "Transform to bird's eye perspective not successful!" << std::endl;
					return;
				}
			}
			else
			{
				// todo: use rescaling with image_scaling_ here
				H = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
				R = H;
				t = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
				camera_image_plane_offset.x = 0.f;
				camera_image_plane_offset.y = 0.f;
				plane_color_image_warped = plane_color_image;
				plane_mask_warped = plane_mask;
			}

			if (debug_["show_plane_color_image"] == true)
			{
				cv::imshow("segmented color image", plane_color_image);
				cvMoveWindow("segmented color image", 650, 0);
				cv::waitKey(10);
			}


			//================== Call ACTION CLIENT =====================================================================================
			//setting up GOALS
			baker_msgs::DirtDetectionGoal goal;
			cv_bridge::CvImage cv_image;
			cv_image.header = point_cloud2_rgb_msg->header;
			cv_image.image = plane_color_image_warped;
			cv_image.encoding = sensor_msgs::image_encodings::BGR8;
			cv_image.toImageMsg(goal.plane_color_image_warped);
			cv_image.image = plane_mask_warped;
			cv_image.encoding = sensor_msgs::image_encodings::MONO8;
			cv_image.toImageMsg(goal.plane_mask_warped);

			//sending goal from this client to the server
			std::cout << "=======================================================\nSending GOAL from client to server." << std::endl;
			dirt_detection_client_.sendGoal(goal);

			//wait for the server to take action and deliver a result
			std::cout << "		Wait for the server to take action and deliver a result." << std::endl;
			bool finished_before_timeout = dirt_detection_client_.waitForResult(ros::Duration(10.0));

			//if not ready before timeout abort
			if (finished_before_timeout == false)
			{
				ROS_ERROR("Timeout on action call to dirt detection.\n");
				return;
			}
			//================== End ACTION CLIENT ======================================================================================

			//check if action was successful
			if (dirt_detection_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				std::cout << "Received RESULT from server.\n" << dirt_detection_client_.getResult()->dirt_detections.size() << "=======================================================" << std::endl;

				cob_object_detection_msgs::DetectionArray detected_dirt_to_publish;
				const cv::Mat H_inv = H.inv();
				for (size_t i=0; i<dirt_detection_client_.getResult()->dirt_detections.size(); ++i) //rescale, find coordinates for, and publish all dirts detected
					{
						// todo: rescale dirt detections with 1./image_scaling_
						const baker_msgs::RotatedRect& det = dirt_detection_client_.getResult()->dirt_detections[i];
						const cv::Rect dirt_warped = cv::RotatedRect(cv::Point2f(det.center_x, det.center_y), cv::Size2f(det.width, det.height), det.angle).boundingRect();
						const int max_v = int(dirt_warped.y + dirt_warped.height*0.5);
						const int max_u = int(dirt_warped.x + dirt_warped.width*0.5);
						for (int v = int(dirt_warped.y - dirt_warped.height*0.5); v <= max_v; ++v)
						{
							for (int u = int(dirt_warped.x - dirt_warped.width*0.5); u <= max_u; ++u)
							{
								cv::Mat point_original_image = (cv::Mat_<double>(3, 1) << u, v, 1.0);
								if (warp_image_ == true)
								{
									point_original_image = H_inv*(cv::Mat_<double>(3, 1) << u, v, 1.0);
									point_original_image.at<double>(0,0) /= point_original_image.at<double>(2,0);
									point_original_image.at<double>(1,0) /= point_original_image.at<double>(2,0);
								}
								const int u_pcl = point_original_image.at<double>(0,0);
								const int v_pcl = point_original_image.at<double>(1,0);
								if (u_pcl >= 0 && u_pcl<input_cloud->width && v_pcl >= 0 && v_pcl<input_cloud->height)
								{
									pcl::PointXYZRGB& point = (*input_cloud)[v_pcl*input_cloud->width + u_pcl];
									// todo: average
								}
							}
						}



						cv::RotatedRect dirt;
						//cv::RotatedRect dirt(cv::Point2f(det.center_x, det.center_y), cv::Size2f(det.width/bird_eye_resolution_, det.height/bird_eye_resolution_), det.angle);
//						if (warp_image_ == true)
//						{
//							H_inv*cv::Point3f();
//						}


						//cv::RotatedRect dirt = dirt_detection_client_.getResult()->dirt_detections[i]; //todo: throws error. need conversion.
						dirt.center.x = dirt.center.x * 1./image_scaling_;
						dirt.center.y = dirt.center.y * 1./image_scaling_;
						dirt.size.width = dirt.size.width * 1./image_scaling_;
						dirt.size.height = dirt.size.height * 1./image_scaling_;

						//if (warp_image_ == true)
						// transform pixel coordinates back by using inverse homography of H

						//todo: find coordinates in pointcloud
						pcl::PointXYZRGB point;
						bool found_point = false;
						for (int v = 0; v < (int) input_cloud->height; v++) //go through point cloud
						{	for (int u = 0; u < (int) input_cloud->width; u++)	//go through point cloud
							{

									for(int i = 0; i < 10; ++i)//check if we find a point in pointcloud thats not a nan and close to coordinate in from dirt_detections rect
									{	for(int j = 0; j < 10; ++j)//check if we find a point in pointcloud thats not a nan and close to coordinate in from dirt_detections rect
										{

											for(int minus_plus = 0; minus_plus < 2; ++minus_plus) //check in both directions
											{
												if(minus_plus==0)
												{
													point = (*input_cloud)[(int)(dirt.size.width * (v+i) + (u+j))];
													if( !std::isnan(point.x) || !std::isnan(point.y) || !std::isnan(point.z))
													{
														found_point = true;
														point = (*input_cloud)[(int)(dirt.size.width * i + j)];
														goto found_point;
													}
												}
												if(minus_plus==1)
												{
													i=-i;
													j=-j;
													point = (*input_cloud)[(int)(dirt.size.width * (v+i) + (u+j))];
													if( !std::isnan(point.x) || !std::isnan(point.y) || !std::isnan(point.z))
													{
														found_point = true;
														point = (*input_cloud)[(int)(dirt.size.width * i + j)];
														goto found_point;
													}
												}
											}

										}
									}

							}
						}
						found_point:; //todo: dont know if its good idea to use goto statement but dont wanted to break multiple for loops.
						//todo: put found point in msgs
						detected_dirt_to_publish.header = point_cloud2_rgb_msg->header;
						detected_dirt_to_publish.detections[i].header = point_cloud2_rgb_msg->header;
						detected_dirt_to_publish.detections[i].label = "dirt_spots_found";

						//todo: error: pose has no member named ‘position'.
//						detected_dirt_to_publish.detections[i].pose.position.x = point.x;
//						detected_dirt_to_publish.detections[i].pose.position.y = point.y;
//						detected_dirt_to_publish.detections[i].pose.position.z = point.z;

//						detected_dirt_to_publish.detections[i].pose.orientation.x;
//						detected_dirt_to_publish.detections[i].pose.orientation.y;
//						detected_dirt_to_publish.detections[i].pose.orientation.z;
//						detected_dirt_to_publish.detections[i].pose.orientation.w;
					}

				//todo: publish coordinates and rects
				dirt_detected_.publish(detected_dirt_to_publish);

				ROS_INFO("Finished IpaDirtDetectionSpectral successfully.\n");
			}
			else
				ROS_ERROR("IpaDirtDetectionSpectral FAILED.\n");
		}
	}
	//publish resulting image with dirt detected
	//dirt_detection_image_pub_.publish();
}

//See header for explanation
bool IpaDirtDetectionPreprocessing::ClientPreprocessing::planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud, const std_msgs::Header& header, cv::Mat& plane_color_image,
		cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, const tf::StampedTransform& transform_map_camera)
{
	//recreate original color image from point cloud
	if (debug_["show_original_image"] == true)
	{
		cv::Mat color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
		int index = 0;
		for (int v = 0; v < (int) input_cloud->height; v++)
		{
			for (int u = 0; u < (int) input_cloud->width; u++, index++)
			{
				pcl::PointXYZRGB& point = (*input_cloud)[index];
				color_image.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, point.r);
			}
		}
		std::cout << input_cloud->height << "x" << input_cloud->width << std::endl;

		//display original image
		cv::imshow("original color image", color_image);
		cvMoveWindow("original color image", 0, 0);
		cv::waitKey();
		if (debug_["save_data_for_test"] == true)
		{
			//bird_eye_resolution_string_ = std::to_string(bird_eye_resolution_);
			cv::imwrite("test_data/ori_image" + bird_eye_resolution_string_ + ".jpg", color_image);
		}
		//cvMoveWindow("color image", 0, 520);
	}

	// try several times to find the ground plane
	double plane_inlier_threshold = 0.05; // cm
	bool found_plane = false;

	// downsample the dataset with a voxel filter using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud(input_cloud->makeShared());
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*filtered_input_cloud);
	//std::cout << "PointCloud after filtering has: " << filtered_input_cloud->points.size ()  << " data points." << std::endl;

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	for (int trial = 0; (trial < floor_search_iterations_ && filtered_input_cloud->points.size() > 100); trial++)
	{
		// Create the segmentation object for the planar model and set all the parameters
		inliers->indices.clear();
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(plane_inlier_threshold);
		seg.setInputCloud(filtered_input_cloud);
		seg.segment(*inliers, plane_model);

		// keep plane_normal upright
		if (plane_model.values[2] < 0.)
		{
			plane_model.values[0] *= -1;
			plane_model.values[1] *= -1;
			plane_model.values[2] *= -1;
			plane_model.values[3] *= -1;
		}

		// verify that plane is a valid ground plane
		if (inliers->indices.size() != 0)//			tf::Vector3 planePointCamera(point.x, point.y, point.z);
			//			tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
			//			//cv::Point2i co(-(planePointWorld.getX()-gridOrigin_.x)*gridResolution_+grid_offset.x, (planePointWorld.getY()-gridOrigin_.y)*gridResolution_+grid_offset.y);	//done: offset
			//			cv::Point2i co((planePointWorld.getX() - gridOrigin_.x) * gridResolution_, (planePointWorld.getY() - gridOrigin_.y) * gridResolution_);

		{
//			tf::StampedTransform rotationMapCamera = transform_map_camera;
//			rotationMapCamera.setOrigin(tf::Vector3(0, 0, 0));
//			tf::Vector3 planeNormalCamera(plane_model.values[0], plane_model.values[1], plane_model.values[2]);
//			tf::Vector3 planeNormalWorld = rotationMapCamera * planeNormalCamera;

//			pcl::PointXYZRGB point = (*filtered_input_cloud)[(inliers->indices[inliers->indices.size() / 2])];
//			tf::Vector3 planePointCamera(point.x, point.y, point.z);
//			tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
//			//std::cout << "normCam: " << planeNormalCamera.getX() << ", " << planeNormalCamera.getY() << ", " << planeNormalCamera.getZ() << "  normW: " << planeNormalWorld.getX() << ", " << planeNormalWorld.getY() << ", " << planeNormalWorld.getZ() << "   point[half]: " << planePointWorld.getX() << ", " << planePointWorld.getY() << ", " << planePointWorld.getZ() << std::endl;

			// verify that the found plane is a valid ground plane
			//if ((int)inliers->indices.size()>min_plane_points_ && planeNormalWorld.getZ()<plane_normal_max_z_ && abs(planePointWorld.getZ())<plane_max_height_)  // add optional check if a preferred plane-normal direction is provided relative to camera and a height of camera above plan
			if ((int) inliers->indices.size() > min_plane_points_)
			{
				found_plane = true;
				break;
			}
			else
			{
//				// the plane is not the ground plane -> remove that plane from the point cloud
//				for (size_t i=0; i<inliers->indices.size(); i++)
//				{
//					// set all inliers to invalid
//					pcl::PointXYZRGB& point = (*filtered_input_cloud)[(inliers->indices[i])];
//					point.x = 0;
//					point.y = 0;
//					point.z = 0;
//				}

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				extract.setInputCloud(filtered_input_cloud);
				extract.setIndices(inliers);

				// Remove the planar inliers, extract the rest
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
				extract.setNegative(true);
				extract.filter(*temp);
				filtered_input_cloud = temp;
			}
		}
	}

	// if the ground plane was found, write the respective data to the images
	if (found_plane == true)
	{
		pcl::PointCloud<pcl::PointXYZRGB> floor_plane;

		// determine all point indices in original point cloud for plane inliers
		inliers->indices.clear();
		for (unsigned int i = 0; i < input_cloud->size(); i++)
		{
			pcl::PointXYZRGB& point = (*input_cloud)[i];
			if (point.x != 0. || point.y != 0. || point.z != 0.)
			{
				// check plane equation
				double distance = plane_model.values[0] * point.x + plane_model.values[1] * point.y + plane_model.values[2] * point.z + plane_model.values[3];
				if (distance > -plane_inlier_threshold && distance < plane_inlier_threshold)
					inliers->indices.push_back(i);
			}
		}

		plane_color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
		plane_mask = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC1);
		std::set<cv::Point2i, lessPoint2i> visitedGridCells; // secures that no two observations can count twice for the same grid cell
//		cv::Point2i grid_offset(0,0);	//(grid_number_observations.cols/2, grid_number_observations.rows/2);	//done: offset
		for (size_t i = 0; i < inliers->indices.size(); i++)
		{
			int v = inliers->indices[i] / input_cloud->width; // todo: check if this rounds down every time
			int u = inliers->indices[i] - v * input_cloud->width;

			// cropped color image and mask
			pcl::PointXYZRGB point = (*input_cloud)[(inliers->indices[i])];
			plane_color_image.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, point.r);
			plane_mask.at<uchar>(v, u) = 255;

			// populate visibility grid
//			tf::Vector3 planePointCamera(point.x, point.y, point.z);
//			tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
//			//cv::Point2i co(-(planePointWorld.getX()-gridOrigin_.x)*gridResolution_+grid_offset.x, (planePointWorld.getY()-gridOrigin_.y)*gridResolution_+grid_offset.y);	//done: offset
//			cv::Point2i co((planePointWorld.getX() - gridOrigin_.x) * gridResolution_, (planePointWorld.getY() - gridOrigin_.y) * gridResolution_);
//			// todo: add a check whether the current point is really visible in the current image of analysis (i.e. the warped image)
//			if (visitedGridCells.find(co) == visitedGridCells.end() && co.x >= 0 && co.x < grid_number_observations.cols && co.y >= 0
//					&& co.y < grid_number_observations.rows)
//			{
//				// grid cell has not been incremented, yet
////				for (std::set<cv::Point2i, lessPoint2i>::iterator it=visitedGridCells.begin(); it!=visitedGridCells.end(); it++)
////				{
////					if (it->x==co.x && it->y==co.y)
////					{
////						std::cout << "co: " << co.x << " " << co.y << std::endl;
////						std::cout << "p:" << it->x << " " << it->y << std::endl;
////						getchar();
////					}
////				}
//				visitedGridCells.insert(co);
//				grid_number_observations.at<int>(co) = grid_number_observations.at<int>(co) + 1;
//			}

//			point.z = -(plane_model.values[0]*point.x+plane_model.values[1]*point.y+plane_model.values[3])/plane_model.values[2];
			floor_plane.push_back(point);
		}
		if (debug_["save_data_for_test"] == true)
		{
			cv::imwrite("test_data/plane_color_image" + bird_eye_resolution_string_ + ".jpg", plane_color_image);
			cv::imwrite("test_data/plane_mask" + bird_eye_resolution_string_ + ".jpg", plane_mask);
		}

		//display detected floor
		//cv::imshow("floor cropped", ground_image);

		// todo: make parameter whether this should be published
		sensor_msgs::PointCloud2 floor_cloud;
		pcl::toROSMsg(floor_plane, floor_cloud);
		floor_cloud.header = header;
		floor_plane_pub_.publish(floor_cloud);

		found_plane = true;
	}

	return found_plane;
}


//See header for explanation
bool IpaDirtDetectionPreprocessing::ClientPreprocessing::computeBirdsEyePerspective(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask,
		pcl::ModelCoefficients& plane_model, cv::Mat& H, cv::Mat& R, cv::Mat& t, cv::Point2f& cameraImagePlaneOffset, cv::Mat& plane_color_image_warped,
		cv::Mat& plane_mask_warped)
{

	// 1. compute parameter representation of plane, construct plane coordinate system and compute transformation from camera frame (x,y,z) to plane frame (x,y,z)
	// a) parameter form of plane equation
	// choose two arbitrary points on the plane
	cv::Point3d p1, p2;
	double a = plane_model.values[0], b = plane_model.values[1], c = plane_model.values[2], d = plane_model.values[3];
	if (a == 0. && b == 0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 1;
		p2.y = 0;
		p2.z = -d / c;
	}
	else if (a == 0. && c == 0.)
	{
		p1.x = 0;
		p1.y = -d / b;
		p1.z = 0;
		p2.x = 1;
		p2.y = -d / b;
		p2.z = 0;
	}
	else if (b == 0. && c == 0.)
	{
		p1.x = -d / a;
		p1.y = 0;
		p1.z = 0;
		p2.x = -d / a;
		p2.y = 1;
		p2.z = 0;
	}
	else if (a == 0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 1;
		p2.y = 0;
		p2.z = -d / c;
	}
	else if (b == 0.)
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 0;
		p2.y = 1;
		p2.z = -d / c;
	}
	else if (c == 0.)
	{
		p1.x = -d / a;
		p1.y = 0;
		p1.z = 0;
		p2.x = -d / a;
		p2.y = 0;
		p2.z = 1;
	}
	else
	{
		p1.x = 0;
		p1.y = 0;
		p1.z = -d / c;
		p2.x = 1;
		p2.y = 0;
		p2.z = (-d - a) / c;
	}
	// compute two normalized directions
	cv::Point3d dirS, dirT, normal(a, b, c);
	double lengthNormal = cv::norm(normal);
//	if (c<0.)
//		lengthNormal *= -1;
	normal.x /= lengthNormal;
	normal.y /= lengthNormal;
	normal.z /= lengthNormal;
	dirS = p2 - p1;
	double lengthS = cv::norm(dirS);
	dirS.x /= lengthS;
	dirS.y /= lengthS;
	dirS.z /= lengthS;
	dirT.x = normal.y * dirS.z - normal.z * dirS.y;
	dirT.y = normal.z * dirS.x - normal.x * dirS.z;
	dirT.z = normal.x * dirS.y - normal.y * dirS.x;
	double lengthT = cv::norm(dirT);
	dirT.x /= lengthT;
	dirT.y /= lengthT;
	dirT.z /= lengthT;

	// b) construct plane coordinate system
	// plane coordinate frame has center p1 and x-axis=dirS, y-axis=dirT, z-axis=normal

	// c) compute transformation from camera frame (x,y,z) to plane frame (x,y,z)
	t = (cv::Mat_<double>(3, 1) << p1.x, p1.y, p1.z);
	R = (cv::Mat_<double>(3, 3) << dirS.x, dirT.x, normal.x, dirS.y, dirT.y, normal.y, dirS.z, dirT.z, normal.z);

//		std::cout << "t: " << p1.x << ", " << p1.y << ", " << p1.z << std::endl;
//		std::cout << "dirS: " << dirS.x << ", " << dirS.y << ", " << dirS.z << std::endl;
//		std::cout << "dirT: " << dirT.x << ", " << dirT.y << ", " << dirT.z << std::endl;
//		std::cout << "normal: " << normal.x << ", " << normal.y << ", " << normal.z << std::endl;

	// 2. select data segment and compute final transformation of camera coordinates to scaled and centered plane coordinates
	std::vector<cv::Point2f> pointsCamera, pointsPlane;
	cv::Point2f minPlane(1e20, 1e20), maxPlane(-1e20, -1e20);
	cv::Mat RTt = R.t() * t;
	for (int v = 0; v < plane_color_image.rows; v++)
	{
		for (int u = 0; u < plane_color_image.cols; u++)
		{
			// black pixels are not part of the plane
			const cv::Vec3b& color = plane_color_image.at<cv::Vec3b>(v, u);
			if (color[0] == 0 && color[1] == 0 && color[2] == 0)
				continue;

			// distance to camera has to be below a maximum distance
			pcl::PointXYZRGB point = (*input_cloud)[v * plane_color_image.cols + u];
			if (point.x * point.x + point.y * point.y + point.z * point.z > max_distance_to_camera_ * max_distance_to_camera_)
				continue;

			// determine max and min x and y coordinates of the plane
			cv::Mat pointCamera = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
			pointsCamera.push_back(cv::Point2f(u, v));
			cv::Mat pointPlane = R.t() * pointCamera - RTt;
			pointsPlane.push_back(cv::Point2f(pointPlane.at<double>(0), pointPlane.at<double>(1)));

			if (minPlane.x > pointPlane.at<double>(0))
				minPlane.x = pointPlane.at<double>(0);
			if (maxPlane.x < pointPlane.at<double>(0))
				maxPlane.x = pointPlane.at<double>(0);
			if (minPlane.y > pointPlane.at<double>(1))
				minPlane.y = pointPlane.at<double>(1);
			if (maxPlane.y < pointPlane.at<double>(1))
				maxPlane.y = pointPlane.at<double>(1);
		}
	}

	// 3. find homography between image plane and plane coordinates
	// a) collect point correspondences
	if (pointsCamera.size() < 100)
		return false;
	double step = std::max(1.0, (double) pointsCamera.size() / 100.0);
	std::vector<cv::Point2f> correspondencePointsCamera, correspondencePointsPlane;
	cameraImagePlaneOffset = cv::Point2f((maxPlane.x + minPlane.x) / 2.f - (image_scaling_ * (double) plane_color_image.cols) / (2 * bird_eye_resolution_),
			(maxPlane.y + minPlane.y) / 2.f - (image_scaling_ * (double) plane_color_image.rows) / (2 * bird_eye_resolution_));
	for (double i = 0; i < (double) pointsCamera.size(); i += step)
	{
		correspondencePointsCamera.push_back(pointsCamera[(int) i]);
		correspondencePointsPlane.push_back(bird_eye_resolution_ * (pointsPlane[(int) i] - cameraImagePlaneOffset));
	}
	// b) compute homography
	H = cv::findHomography(correspondencePointsCamera, correspondencePointsPlane);		// i.e. H*camera = plane
//		correspondencePointsCamera.push_back(cv::Point2f(160,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(320,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(480,400));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(160,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(320,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		correspondencePointsCamera.push_back(cv::Point2f(480,200));
//		correspondencePointsPlane.push_back(cv::Point2f(0,0));
//		for (int i=0; i<(int)correspondencePointsCamera.size(); i++)
//		{
//			cv::Mat pc = (cv::Mat_<double>(3,1) << (double)correspondencePointsCamera[i].x, (double)correspondencePointsCamera[i].y, 1.0);
//			cv::Mat pp = (cv::Mat_<double>(3,1) << (double)correspondencePointsPlane[i].x, (double)correspondencePointsPlane[i].y, 1.0);
//
//			cv::Mat Hp = H.inv()*pp;
//			Hp.at<double>(0) /= Hp.at<double>(2);
//			Hp.at<double>(1) /= Hp.at<double>(2);
//			Hp.at<double>(2) = 1.0;
//			cv::Mat r = pc - Hp;
//
//			std::cout << "H - " << i << ": " << r.at<double>(0) << ", " << r.at<double>(1) << ", " << r.at<double>(2) << std::endl;
//		}

	// 4. warp perspective
	cv::Size target_size(image_scaling_ * plane_color_image.cols, image_scaling_ * plane_color_image.rows);
	cv::warpPerspective(plane_color_image, plane_color_image_warped, H, target_size);
	// todo: better manual sampling of the warped mask needed
	cv::warpPerspective(plane_mask, plane_mask_warped, H, target_size);

//		// this example is correct, H transforms world points into the image coordinate system
//		std::vector<cv::Point2f> c1, c2;
//		c1.push_back(cv::Point2f(885,1362));
//		c2.push_back(cv::Point2f(0,0));
//		c1.push_back(cv::Point2f(880,1080));
//		c2.push_back(cv::Point2f(0, 142.7));
//		c1.push_back(cv::Point2f(945,1089));
//		c2.push_back(cv::Point2f(83.7,142.7));
//		c1.push_back(cv::Point2f(948,1350));
//		c2.push_back(cv::Point2f(83.7,0));
//		cv::Mat Hcc = cv::findHomography(c2, c1);
//		std::cout << "H: " << std::endl;
//		for (int v=0;v<3; v++)
//		{
//			for (int u=0; u<3; u++)
//				std::cout << Hcc.at<double>(v,u) << "\t";
//			std::cout << std::endl;
//		}
//		// add test points
//		c1.push_back(cv::Point2f(1010, 1338));
//		c2.push_back(cv::Point2f(167.4,0));
//		c1.push_back(cv::Point2f(1010, 1098));
//		c2.push_back(cv::Point2f(167.4,142.7));
//		for (int i=0; i<(int)c1.size(); i++)
//		{
//			cv::Mat pc = (cv::Mat_<double>(3,1) << (double)c1[i].x, (double)c1[i].y, 1.0);
//			cv::Mat pp = (cv::Mat_<double>(3,1) << (double)c2[i].x, (double)c2[i].y, 1.0);
//			cv::Mat Hccpp = Hcc*pp;
//			Hccpp.at<double>(0) /= Hccpp.at<double>(2);
//			Hccpp.at<double>(1) /= Hccpp.at<double>(2);
//			Hccpp.at<double>(2) = 1.0;
//			cv::Mat r = pc - Hccpp;
//			std::cout << "H - " << i << ": " << r.at<double>(0) << ", " << r.at<double>(1) << ", " << r.at<double>(2) << std::endl;
//		}

	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "dirt_detection_client_preprocessing");
	ros::NodeHandle nh;

	IpaDirtDetectionPreprocessing::ClientPreprocessing client_preprocessing(nh);

	ros::spin();

	return 0;
}
