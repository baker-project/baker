#ifndef FEATURE_EXTRACT_H_
#define FEATURE_EXTRACT_H_
#include <boost/iterator/iterator_concepts.hpp>
#include <ipa_dirt_detection/filterbank_create.h>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <opencv2/ml/ml.hpp>
#include <cv_bridge/cv_bridge.h>

namespace  ipa_FeatureExtract
{
  class FeatureExtract
  {
  private:
    ros::NodeHandle nh_;
    cv::Mat input_image_;
    cv::Mat a_, b_, brightness_;
    
    double sigma_base_;
    int orients_;                // number of orientations for filter kernels
    int elong_;
    int scales_;
    
  public:
    FeatureExtract(ros::NodeHandle nh);
    ~FeatureExtract();
    void init();
    
    void rgbToCIE();
    void preProcessing();
    
  };
};

#endif