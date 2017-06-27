/*
* Copyright (c) 2016-2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

// opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <baker_msgs/GetMap.h>

class MapManagementClient
{
public:
	MapManagementClient(ros::NodeHandle nh) : node_handle_(nh)
	{
		// receive map
		map_data_recieved_ = false;
		map_msg_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &MapManagementClient::mapDataCallback, this);
		ROS_INFO("MapManagementClient: Waiting to receive map...");
		while (map_data_recieved_ == false)
		{
			ros::Duration(0.1).sleep();
			ros::spinOnce();
		}
		ROS_INFO("MapManagementClient: Map received.");

		// advertise service for the map
		get_map_server_ = nh.advertiseService("get_map_image", &MapManagementClient::getMapCallback, this);
	}

protected:

	void mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
	{
		map_header_ = map_msg_data->header;
		map_resolution_ = map_msg_data->info.resolution;
		map_origin_ = map_msg_data->info.origin;
		ROS_INFO_STREAM("MapManagementClient::mapDataCallback: map resolution: " << map_msg_data->info.resolution);

		// create original map
		map_ = 255 * cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);
		for (unsigned int v = 0, i = 0; v < map_msg_data->info.height; v++)
		{
			for (unsigned int u = 0; u < map_msg_data->info.width; u++, i++)
			{
				if (map_msg_data->data[i] != 0)
					map_.at<unsigned char>(v, u) = 0;
			}
		}
//		cv::imshow("map", map_);
//		cv::waitKey();

		map_data_recieved_ = true;
		map_msg_sub_.shutdown();
	}

	bool getMapCallback(baker_msgs::GetMap::Request& req, baker_msgs::GetMap::Response& res)
	{
		// return the map data
		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = map_;
		cv_ptr.encoding = "mono8";
		res.map = *(cv_ptr.toImageMsg());
		res.map.header = map_header_;
		res.map_origin = map_origin_;
		res.map_resolution = map_resolution_;
		return true;
	}

	ros::NodeHandle node_handle_;

	ros::Subscriber map_msg_sub_;		// subscriber to the map topic
	bool map_data_recieved_;
	cv::Mat map_;						// format CV_8UC1, obstacles=0, free space=255
	double map_resolution_;				// in [m/cell]
	geometry_msgs::Pose map_origin_;	// in [m]
	std_msgs::Header map_header_;

	ros::ServiceServer get_map_server_;		// server handling requests for obtaining the current map in image format
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_management_client");
	ros::NodeHandle nh;

	MapManagementClient mmc(nh);

	ros::spin();

	return 0;
}
