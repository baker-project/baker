#ifndef DATASET_CREATE_H_
#define DATASET_CREATE_H_

#include <set>
#include <iostream>
#include <string.h>
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

//boost
#include <boost/foreach.hpp>

namespace ipa_DatasetCreate
{
  class DatasetCreate
  {
  protected:
    ros::NodeHandle node_handle_;
    
    ros::Subscriber camera_depth_points_sub_;
    
  public:
    DatasetCreate(ros::NodeHandle node_handle);
    ~DatasetCreate();
    void init();
    void datasetCreateCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);
    void pointcloudRosTOPointcloudPcl(const sensor_msgs::PointCloud2::ConstPtr& point_cloud2_rgb_msg, 
				      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_XYZRGB);
    bool planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, 
			   const tf::StampedTransform& transform_map_camera, cv::Mat& grid_number_observations);
    int floorSearchIterations_;
    std::string rgbImageSavePath_;
    std::string dirtObjectSavePath_;
    cv::Point2d gridOrigin_;
    double gridResolution_;
    cv::Mat gridNumberObservations_;
    cv::Point2i gridDimensions_;	// number of grid cells in x and y direction = width and height [in number grid cells]
    cv::Mat gridPositiveVotes_;		// grid map that counts the positive votes for dirt
    int minPlanePoints_;
    double distanceToCamera_;
    bool groundSegmentation_;
    
  private:
    	struct bgr
	{
		uchar b; /**< Blue channel value. */
		uchar g; /**< Green channel value. */
		uchar r; /**< Red channel value. */
	};
        int frame_counter_;
  };
};

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
};


#endif