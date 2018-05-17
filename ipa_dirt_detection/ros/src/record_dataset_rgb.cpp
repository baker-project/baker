#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ipa_dirt_detection/record_dataset_rgb.h>

using namespace ipa_DatasetCreate;


DatasetCreate::DatasetCreate(ros::NodeHandle node_handle)
:node_handle_(node_handle)
{
}

DatasetCreate::~DatasetCreate()
{
}

void DatasetCreate::init()
{
  frame_counter_ = 0;
  camera_rgb_image_sub_ = node_handle_.subscribe("/camera/rgb/image_raw", 1, &DatasetCreate::imageCallback, this);
}

 
void DatasetCreate::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
     int key = 0;
     key = cv::waitKey(100);
     
     if (key > 0)
     {
       std::cout  << "save an image " << std::endl;
       cv::imwrite("/home/robot/ground/image" + patch::to_string(frame_counter_) + ".png", cv_bridge::toCvShare(msg, "bgr8")->image);
       frame_counter_++;
     }
     
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_dataset");
  ros::NodeHandle nh;
  
  DatasetCreate dc(nh);
  dc.init();
  
  ros::spin();

}
