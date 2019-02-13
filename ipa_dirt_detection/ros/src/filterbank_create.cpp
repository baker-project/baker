/*
 * The code used to extracted mPb features from the training input frames
 * 
 */


#include <ipa_dirt_detection/filterbank_create.h>

using namespace cv;
using namespace std;
using namespace ipa_FeatureExtract;

/*
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
  FeatureExtract::derivOris();
  for (double orin = 0; orin < num_ori_; orin++)
  {
    gaussianFilter2D(oris[orin],sigma_,0,3);
  }
  
}

void ipa_FeatureExtract::FeatureExtract::gaussianFilter2D(double ori, float sigma,
							  bool hlbrt, double elongation)
{
  int support = ceil(3 * sigma);          // kernel size of the gaussian filter, gaussian function appeals to zero after 3 times of sigma
  double sigma_x = sigma;
  double sigma_y = sigma_x / elongation;
  cv::Mat kernel;
  
  std::vector<double> filter1D_x, filter1D_y;
  int support_x_rotate = support_x_rotate(support, support, ori);
  int support_y_rotate = support_y_rotate(support, support, ori);
  gaussianFilter1D(sigma_x, 0, 0, support_x_rotate, filter1D_x);        // TODO check here the deriv and sigma
  gaussianFilter1D(sigma_y, 0, 0, support_y_rotate, filter1D_y);
  
  // rotate based on the orientation
  for (int x_n = 0; x_n < filter1D_x.size(); x_n++)
  {
    for (int y_n = 0; y_n < filter1D_y.size(); y_n++)
    {
      kernel.at<double>(y_n, x_n) = filter1D_x(x_n) * filter1D_y(y_n);
    }
  }
  filter_bank.push_back(kernel);
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


void FeatureExtract::rotateKernel(double ori, cv::Mat& kernel)
{
  double cos_ori = cos(ori);
  double sin_ori = sin(ori);
  
  int kernel_rows = kernel.rows;
  int kernel_cols = kernel.cols; 
  double rows_ori = static_cast <double>(kernel_rows - 1) / 2;
  double cols_ori = static_cast <double>(kernel_cols - 1) / 2;
  
  
}


double FeatureExtract::supportXRotate(double support_x, double support_y, double ori)
{
   const double sx_cos_ori = support_x * cos(ori);
   const double sy_sin_ori = support_y * sin(ori);
   double x0_mag = abs(sx_cos_ori - sy_sin_ori);
   double x1_mag = abs(sx_cos_ori + sy_sin_ori);
   return (x0_mag > x1_mag) ? x0_mag : x1_mag;
}


double FeatureExtract::supportYRotate(double support_x, double support_y, double ori)
{
   const double sx_sin_ori = support_x * sin(ori);
   const double sy_cos_ori = support_y * cos(ori);
   double y0_mag = abs(sx_sin_ori - sy_cos_ori);
   double y1_mag = abs(sx_sin_ori + sy_cos_ori);
   return (y0_mag > y1_mag) ? y0_mag : y1_mag;
}



void ipa_FeatureExtract::FeatureExtract::derivOris()
{
  double ori_step = cv::CV_PI / num_ori_;
  for (int i = 0; i < num_ori_; i++)
  {
    oris[i] = i * ori_step;
  }
}
*/


FilterbankCreate::FilterbankCreate(int orients, int scales, float sigma_base,double elong)
{
  orients_ = orients;                // number of orientations of the filters
  scales_ = scales;                  // number of scales
  sigma_base_ = sigma_base;          // start sigma for gaussian kernel
  elong_ = elong;
  pi_ = 3.1415926;
  support_ = 3;                         // gaussian kernel is approximately be zeros at three times of sigma
  mean_ = 0;
}


void FilterbankCreate::createKernel(cv::Mat even_kernel, cv::Mat odd_kernel)
{
  int i = 0, j = 0;
  for (int y = (int) hsz_; y >= (int) hsz_; y--)                                
  {
    for ( int x = -(int) hsz_; x <= (int) hsz_; x++ )
    {
      double u = c_ * x - s_ * y;
      double v = s_ * x + c_ * y;
      double g_u = gaussianKernel(sigma_base_ * elong_, u, 0);
      double g_v_even = gaussianKernel(sigma_base_ , v , 1);
      double g_v_odd = gaussianKernel(sigma_base_, v, 2);
      even_kernel.at<double>(i,j) = g_u * g_v_even;            // TODO check index
      odd_kernel.at<double>(i,j) = g_u * g_v_odd;
      i++;
    }
    j++;
  }
  // TODO NORMALISE THE KERNEL
}


double FilterbankCreate::gaussianKernel(double sigma, double uv, int ord)
{
   uv = uv - mean_;
   double num_uv = uv * uv;
   double variance = sigma * sigma;
   double denom = 2 * variance;
   double g_uv=exp(-num_uv/denom) / pow((pi_*denom),0.5);
   if (ord == 1)
   {
      g_uv=-g_uv * (uv/variance);
      return g_uv;
   }
   if (ord == 2)
   {
      g_uv=g_uv * ((num_uv-variance) / pow(variance,2));
      return g_uv;
   }
   return g_uv;
}


void FilterbankCreate::centerSorroundKernel(Mat center_sorround_kernel)
{
  // It is actually a 2D gaussian filter
  double var = 2;
  for (int m = -hsz_ ; m <= hsz_; m++)
  {
    int i = m + hsz_;
    for (int n = -hsz_; n <= hsz_; n++)
    {
      int j = n + hsz_;
      double g = ( 1 / sqrt( 2*pi_ * var )) * exp(-(m * m + n * n ) / (2*var));
      center_sorround_kernel.at<double>(i,j) = g;
    }
  }
}



void FilterbankCreate::createFilterBank(std::vector<cv::Mat>& filter_bank)
{
  hsz_ = ceil(sigma_base_ * support_);              // half size of the filter kernel
  sz_ = (int) ( hsz_ * 2 + 1);                      // size of the filter kernel
  
  for (int ori = 0; ori < orients_; ori++)
  {
    double orient = pi_ / orients_ * ori;
    s_ = sin(orient); c_ = cos(orient);
    cv::Mat odd_kernel, even_kernel;
    createKernel(even_kernel, odd_kernel);                 // create even and odd filter kernel of given orientation
    filter_bank.push_back(even_kernel);
    filter_bank.push_back(odd_kernel);
  }
  cv::Mat center_sorround_kernel;
  centerSorroundKernel(center_sorround_kernel);
  filter_bank.push_back(center_sorround_kernel);
}


/*
int main()
{
  int orients = 8;
  double sigma_base = 2.0;
  int elong = 3;
  int scales = 1;
  FilterbankCreate filter_bank_creator(orients, scales, sigma_base,elong);
  
  std::vector<cv::Mat> filter_bank;
  filter_bank_creator.createFilterBank(filter_bank);
  std::cout << "There are " << filter_bank.size() << " filters" << std::endl; 
}
*/