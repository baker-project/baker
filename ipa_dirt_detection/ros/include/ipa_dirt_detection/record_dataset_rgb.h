#ifndef DATASET_CREATE_H_
#define DATASET_CREATE_H_

#include <set>
#include <iostream>
#include <string>
#include <functional>
#include <fstream>
#include <vector>
#include <deque>
#include <math.h>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/ml/ml.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

//boost
#include <boost/foreach.hpp>

namespace ipa_DatasetCreate
{
class DatasetCreate
{
protected:
	ros::NodeHandle node_handle_;

	ros::Subscriber camera_rgb_image_sub_;

	std::string base_path_;

	int current_floor_index_;
	int current_image_index_;

	void addZerosToFileName(std::stringstream& name, const int index);

	void selectNextFreeFloorIndex();
	void selectNextFreeImageIndex();

public:
	DatasetCreate(ros::NodeHandle node_handle);
	~DatasetCreate();
	void init();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
	struct bgr
	{
		uchar b; /**< Blue channel value. */
		uchar g; /**< Green channel value. */
		uchar r; /**< Red channel value. */
	};
	int frame_counter_;
};
}
;

namespace patch
{
template<typename T> std::string to_string(const T& n)
{
	std::ostringstream stm;
	stm << n;
	return stm.str();
}
}
;

#endif
