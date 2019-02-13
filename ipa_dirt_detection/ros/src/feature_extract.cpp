#include <iostream>
#include <ipa_dirt_detection/feature_extract.h>

using namespace ipa_FeatureExtract;
using namespace std;
using namespace cv;

void FeatureExtract::init()
{
  nh_.param("feature_extract/orients", orients_, 8);
  std::cout << "There are " << orients_ << "orientations of the filter." << std::endl;
  nh_.param("feature_extract/sigma_base", sigma_base_, 1.0);
  std::cout << "Sigma of gaussian filter is " << sigma_base_ << std::endl;
  nh_.param("feature_extract/elong", elong_, 3);
  std::cout << "Elong is " << elong_ << std::endl;
  nh_.param("feature_extract/scales", scales_, 1);
  std::cout << "The scales for filterbank is " << scales_ << std::endl;
  
  // TODO Here to receive the message frames from the dataset
}


void FeatureExtract::rgbToCIE()
{
  cv::Mat cie;
  cv::cvtColor(input_image_, cie, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(cie ,channels);
  
  brightness_ = channels[0];
  a_ = channels[1];
  b_ = channels[2];
}

void FeatureExtract::preProcessing()
{
  
}






int main(int argc,char** argv)
{
  ros::init(argc, argv, "feature_extract");
  ros::NodeHandle n;
  
  int orients = 8;
  double sigma_base = 2.0;
  int elong = 3;
  int scales = 1;
  FilterbankCreate filter_bank_creator(orients, scales, sigma_base,elong);
  std::vector<cv::Mat> filter_bank;
  filter_bank_creator.createFilterBank(filter_bank);
  std::cout << "There are " << filter_bank.size() << " filters" << std::endl; 
  
  ros::spin();
  return 0;
}