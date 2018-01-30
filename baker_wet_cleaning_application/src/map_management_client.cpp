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
		// receive maps
		map_data_recieved_ = false;
		map_msg_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &MapManagementClient::mapDataCallback, this);
		map_segmented_data_recieved_ = false;
		map_segmented_msg_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map_segmented", 1, &MapManagementClient::mapSegmentedDataCallback, this);
		ROS_INFO("MapManagementClient: Waiting to receive map...");
		while (map_data_recieved_ == false)
		{
			ros::Duration(0.1).sleep();
			ros::spinOnce();
		}
		ROS_INFO("MapManagementClient: Map received.");

		// advertise service for the maps
		get_map_server_ = nh.advertiseService("get_map_image", &MapManagementClient::getMapCallback, this);
		get_map_segmented_server_ = nh.advertiseService("get_map_segmented_image", &MapManagementClient::getMapSegmentedCallback, this);
	}

protected:

	void mapDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
	{
		map_header_ = map_msg_data->header;
		map_resolution_ = map_msg_data->info.resolution;
		map_origin_ = map_msg_data->info.origin;
		ROS_INFO_STREAM("MapManagementClient::mapDataCallback: map resolution: " << map_resolution_
				<< "    map_origin: (" << map_origin_.position.x << ", " << map_origin_.position.y << ")");

		// create original map
		map_ = 255 * cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);
		for (unsigned int v = 0, i = 0; v < map_msg_data->info.height; v++)
		{
			for (unsigned int u = 0; u < map_msg_data->info.width; u++, i++)
			{
				if (map_msg_data->data[i] > 100. * 100./255.)		// accessible areas are <=100 in MIRA (value range 0...255), whereas ROS has a value range of 0...100
					map_.at<unsigned char>(v, u) = 0;
			}
		}
//		cv::Mat temp = map_;
		// do not flip
		//cv::flip(temp, map_, 0);	// without flip, you see the data arrangement in the OccupancyGrid message, flipped you see the map as is in RViz,
									// with flipping, the coordinate systems between OccupancyGrid (lower left origin) and image (upper left origin) do not match anymore
//		cv::imshow("mapDataCallback_map", map_);
//		cv::waitKey();

		map_data_recieved_ = true;
		map_msg_sub_.shutdown();
	}

	bool getMapCallback(baker_msgs::GetMap::Request& req, baker_msgs::GetMap::Response& res)
	{
		if (map_data_recieved_==false)
			return false;

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

	void mapSegmentedDataCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg_data)
	{
		map_segmented_header_ = map_msg_data->header;
		map_segmented_resolution_ = map_msg_data->info.resolution;
		map_segmented_origin_ = map_msg_data->info.origin;
		ROS_INFO_STREAM("MapManagementClient::mapSegmentedDataCallback: map segmented resolution: " << map_segmented_resolution_
				<< "    map_segmented_origin: (" << map_segmented_origin_.position.x << ", " << map_segmented_origin_.position.y << ")");

		// create original map
		map_segmented_ = 255 * cv::Mat::ones(map_msg_data->info.height, map_msg_data->info.width, CV_8UC1);
		for (unsigned int v = 0, i = 0; v < map_msg_data->info.height; v++)
		{
			for (unsigned int u = 0; u < map_msg_data->info.width; u++, i++)
			{
				if (map_msg_data->data[i] > 100. * 100./255.)		// accessible areas are <=100 in MIRA (value range 0...255), whereas ROS has a value range of 0...100
					map_segmented_.at<unsigned char>(v, u) = 0;
			}
		}
//		cv::Mat temp = map_segmented_;
		// do not flip
		//cv::flip(temp, map_, 0);	// without flip, you see the data arrangement in the OccupancyGrid message, flipped you see the map as is in RViz,
									// with flipping, the coordinate systems between OccupancyGrid (lower left origin) and image (upper left origin) do not match anymore
//		cv::imshow("mapDataCallback_map", map_);
//		cv::waitKey();

		map_segmented_data_recieved_ = true;
		map_segmented_msg_sub_.shutdown();
	}

	bool getMapSegmentedCallback(baker_msgs::GetMap::Request& req, baker_msgs::GetMap::Response& res)
	{
		if (map_segmented_data_recieved_==false)
			return false;

		// return the map data
		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = map_segmented_;
		cv_ptr.encoding = "mono8";
		res.map = *(cv_ptr.toImageMsg());
		res.map.header = map_segmented_header_;
		res.map_origin = map_segmented_origin_;
		res.map_resolution = map_segmented_resolution_;
		return true;
	}

	ros::NodeHandle node_handle_;

	// map storage
	ros::Subscriber map_msg_sub_;		// subscriber to the map topic
	bool map_data_recieved_;
	cv::Mat map_;						// format CV_8UC1, obstacles=0, free space=255
	double map_resolution_;				// in [m/cell]
	geometry_msgs::Pose map_origin_;	// in [m]
	std_msgs::Header map_header_;

	// map segmented storage
	ros::Subscriber map_segmented_msg_sub_;		// subscriber to the map segmented topic
	bool map_segmented_data_recieved_;
	cv::Mat map_segmented_;						// format CV_8UC1, obstacles=0, free space=255
	double map_segmented_resolution_;			// in [m/cell]
	geometry_msgs::Pose map_segmented_origin_;	// in [m]
	std_msgs::Header map_segmented_header_;

	ros::ServiceServer get_map_server_;		// server handling requests for obtaining the current map in image format
	ros::ServiceServer get_map_segmented_server_;		// server handling requests for obtaining a manually or externally segmented map in image format
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_management_client");
	ros::NodeHandle nh("~");

	MapManagementClient mmc(nh);

	ros::spin();

	return 0;
}
