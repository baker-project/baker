/*
 * The code used to extracted mPb features from the training input frames
 * 
 */


#include <ipa_dirt_detection/feature_extract.h>

using namespace cv;
using namespace std;
using namespace ipa_FeatureExtract;

ipa_FeatureExtract::FeatureExtract::FeatureExtract(std::string input_path)
{
  input_image_ = cv::imread(input_path);
}

ipa_FeatureExtract::FeatureExtract::~FeatureExtract()
{

}

void ipa_FeatureExtract::FeatureExtract::preProcessing()
{
  
}

void ipa_FeatureExtract::FeatureExtract::rgbToCIE()
{
  cv::Mat cie;
  cv::cvtColor(input_image_, cie, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(cie ,channels);
  
  brightness_ = channels[0];
  a_ = channels[1];
  b_ = channels[2];
}

void ipa_FeatureExtract::FeatureExtract::filterBank()
{
  
}

void ipa_FeatureExtract::FeatureExtract::gaussianFilter2D(int ori, float sigma, int deriv,
							  bool hlbrt, double elongation)
{
  int support = ceil(3 * sigma);          // kernel size of the gaussian filter, gaussian function appeals to zero after 3 times of sigma
  double sigma_x = sigma;
  double sigma_y = sigma_x / elongation;
  
  std::vector<double> filter1D_x, filter1D_y;
  gaussianFilter1D(sigma_x, 0, 0, support, filter1D_x);        // TODO check here the deriv and sigma
  gaussianFilter1D(sigma_y, 0, 0, support, filter1D_y);
  
  // rotate based on the orientation
}

void ipa_FeatureExtract::FeatureExtract::gaussianFilter1D(double sigma, int deriv, 
							  bool hlbrt, int support, 
							  std::vector<double>& filter1D)
{
  double sigma2_inv = double(1)/(sigma*sigma);
  double neg_two_sigma2_inv = double(-0.5)*sigma2_inv;
  
  int size = 2 * support + 1;
  double x = -static_cast<double> (support);
  double mean = 0;
  double kernel_value = 0;
  
    if (deriv == 0)
    {
       for (int i = 0; i < size; i++)
       {
	 kernel_value = exp(x * x * neg_two_sigma2_inv);
	 filter1D.push_back(kernel_value);
	 mean += kernel_value;
      }
    }
    else if(deriv == 1)
    {
       for (int i = 0; i < size; i++)
       {
	 kernel_value = exp(x * x * neg_two_sigma2_inv)*(-x);
	 filter1D.push_back(kernel_value);
	 mean += kernel_value;
      }
    }
    else if(deriv == 2)
    {
      for (int i = 0; i < size; i++)
      {
	double x2 = x * x;
	kernel_value = exp(x2*neg_two_sigma2_inv) * (x2*sigma2_inv - 1);
	filter1D.push_back(kernel_value);
	mean += kernel_value;
      }
    }
    mean /= size;
    
    // TODO hilbert transform
    
    // normalization
    std::transform(filter1D.begin(),filter1D.end(), filter1D.begin(),
                   std::bind2nd(std::minus<double>(),mean));
    std::vector<double> filter_abs;
    std::transform(filter1D.begin(), filter1D.end(),
		   filter_abs.begin(),std::abs());
    double abs_sum = 0;
    for (auto& n : filter_abs)
      abs_sum += n;
    std::transform(filter1D.begin(),filter1D.end(), filter1D.begin(),
                   std::bind2nd(std::divides<double>(),abs_sum));
}


void ipa_FeatureExtract::FeatureExtract::derivOris()
{
  double ori_step = cv::CV_PI / num_ori_;
  for (int i = 0; i < num_ori_; i++)
  {
    oris[i] = i * ori_step;
  }
}



