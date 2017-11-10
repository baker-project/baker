/*
 * Main cpp code for dataset creation for dirt detection
 * Dataset of the clean ground and the manually added dirt  
 */

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <functional>

#include <ipa_dirt_detection/dataset_create.h>

using namespace ipa_DatasetCreate;
using namespace std;
using namespace cv;

struct lessPoint2i : public std::binary_function<cv::Point2i, cv::Point2i, bool>
{
	bool operator()(const cv::Point2i& a, const cv::Point2i& b) const
	{ return ((a.x<b.x) || ((a.x==b.x) && (a.y<b.y))); }
};

DatasetCreate::DatasetCreate(ros::NodeHandle node_handle)
:node_handle_(node_handle)
{
  
}

DatasetCreate::~DatasetCreate()
{  
}

void DatasetCreate::init()
{
  node_handle_.param("dataset_create/floorSearchIterations", floorSearchIterations_, 3);
  std::cout << "floorSearchIterations is " << floorSearchIterations_ << std::endl;
  
  node_handle_.param<std::string>("dataset_create/rgbImageSavePath", rgbImageSavePath_, "test_data");
  std::cout << "RGB Image save path is " << rgbImageSavePath_ << std::endl;
  node_handle_.param<std::string>("dataset_create/dirtObjectSavePath", dirtObjectSavePath_, "test_data");
  std::cout << "Dirt Objects save path is " << dirtObjectSavePath_ << std::endl;
  
  double gox, goy;
  node_handle_.param("dataset_create/gridOrigin_x", gox, 0.0);
  std::cout << "gridOrigin_x = " << gox << std::endl;
  node_handle_.param("dataset_create/gridOrigin_y", goy, 0.0);
  std::cout << "gridOrigin_y = " << goy << std::endl;
  gridOrigin_ = cv::Point2d(gox, goy);
  node_handle_.param("dataset_create/gridResolution", gridResolution_, 20.0);
  std::cout << "gridResolution = " << gridResolution_ << std::endl;
  int gdx, gdy;
  node_handle_.param("dataset_create/gridDimensions_x", gdx, 100);
  std::cout << "gridDimensions_x = " << gdx << std::endl;
  node_handle_.param("dataset_create/gridDimensions_y", gdy, 100);
  std::cout << "gridDimensions_y = " << gdy << std::endl;
  gridDimensions_ = cv::Point2i(gdx, gdy);
  node_handle_.param("dataset_create/minPlanePoints", minPlanePoints_, 100);
  std::cout << "minPlanePoints = " << minPlanePoints_ << std::endl;
  node_handle_.param("dataset_create/distanceToCamera", distanceToCamera_, 1.0);
  std::cout << "The distance between camera and object = " << distanceToCamera_ << std::endl;
  
  frame_counter_ = 0;
  camera_depth_points_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("colored_point_cloud", 1, &DatasetCreate::datasetCreateCallback, this);
}


void ipa_DatasetCreate::DatasetCreate::datasetCreateCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg)
{
  std::cout << "receive a frame" << std::endl;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pointcloudRosTOPointcloudPcl(point_cloud2_rgb_msg, input_cloud);
  
  // get tf between camera and map
  tf::StampedTransform transformMapCamera;
  transformMapCamera.setIdentity();
  
  gridPositiveVotes_ = cv::Mat::zeros(gridDimensions_.y, gridDimensions_.x, CV_32SC1);
  gridNumberObservations_ = cv::Mat::zeros(gridPositiveVotes_.rows, gridPositiveVotes_.cols, CV_32SC1);
  
  cv::Mat plane_color_image;
  cv::Mat plane_mask;
  pcl::ModelCoefficients plane_model;
  bool found_plane = planeSegmentation(input_cloud, plane_color_image, plane_mask, plane_model, transformMapCamera, gridNumberObservations_);
  
  if (found_plane == true)
  {
    std::cout << "size of the image is " << plane_color_image.cols << std::endl;
    cv::imshow("plane_color_image", plane_color_image);
    cv::waitKey(10);
    frame_counter_ ++;
    cv::imwrite(rgbImageSavePath_ + '/' + string(frame_counter_) + '.jpg', plane_color_image);
  }
  else
  {
     std::cout << "no plane found " << std::endl;
  }
  
  
}


void DatasetCreate::pointcloudRosTOPointcloudPcl(const sensor_msgs::PointCloud2_< std::allocator< void > >::ConstPtr& point_cloud2_rgb_msg, 
								pcl::PointCloud< pcl::PointXYZRGB >::Ptr& point_cloud_XYZRGB)
{
  pcl::fromROSMsg(*point_cloud2_rgb_msg, *point_cloud_XYZRGB);
}


bool DatasetCreate::planeSegmentation(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input_cloud, cv::Mat& plane_color_image, 
				      cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, const tf::StampedTransform& transform_map_camera, cv::Mat& grid_number_observations)
{

   cv::Mat color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
   int index = 0;
   for (int v=0; v<(int)input_cloud->height; v++)
   {
     for (int u=0; u<(int)input_cloud->width; u++, index++)
     {
       pcl::PointXYZRGB point = (*input_cloud)[index];
       bgr bgr_ = {point.b, point.g, point.r};
       color_image.at<bgr>(v, u) = bgr_;
     }
   }	

   // try several times to find the ground plane
   double plane_inlier_threshold = 0.01;	// 0.05 default
   bool found_plane = false;

   // downsample the dataset with a voxel filter using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PointXYZRGB> vg;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   vg.setInputCloud(input_cloud->makeShared());
   vg.setLeafSize(0.01f, 0.01f, 0.01f);
   vg.filter(*filtered_input_cloud);
   
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
   for (int trial=0; (trial<floorSearchIterations_ && filtered_input_cloud->points.size()>100); trial++)
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
	if (inliers->indices.size()!=0)
	{
	 tf::StampedTransform rotationMapCamera = transform_map_camera;
	 rotationMapCamera.setOrigin(tf::Vector3(0,0,0));
	 tf::Vector3 planeNormalCamera(plane_model.values[0], plane_model.values[1], plane_model.values[2]);
	 tf::Vector3 planeNormalWorld = rotationMapCamera * planeNormalCamera;

	 pcl::PointXYZRGB point = (*filtered_input_cloud)[(inliers->indices[inliers->indices.size()/2])];
	 tf::Vector3 planePointCamera(point.x, point.y, point.z);
	 tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
	 
	  if ((int)inliers->indices.size()>minPlanePoints_)
	  {
	   found_plane=true;
	    break;
	  }
	}

	else
	{
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
     }  // end of the plane search

	// if the ground plane was found, write the respective data to the images
	if (found_plane == true)
	{
	  inliers->indices.clear();
	  for (unsigned int i=0; i<input_cloud->size(); i++)
	  {
	    pcl::PointXYZRGB& point = (*input_cloud)[i];
	    if (point.x!=0. || point.y!=0. || point.z!=0.)
	    {
	      double distance = plane_model.values[0]*point.x + plane_model.values[1]*point.y + plane_model.values[2]*point.z + plane_model.values[3];
	      if (distance > -plane_inlier_threshold && distance < plane_inlier_threshold)
		inliers->indices.push_back(i);
	    }
	 }

	plane_color_image = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC3);
	plane_mask = cv::Mat::zeros(input_cloud->height, input_cloud->width, CV_8UC1);
	std::set<cv::Point2i, lessPoint2i> visitedGridCells;	// secures that no two observations can count twice for the same grid cell

	for (size_t i=0; i<inliers->indices.size(); i++)
	{
	  int v = inliers->indices[i]/input_cloud->width;	
	  int u = inliers->indices[i] - v*input_cloud->width;

	  // cropped color image and mask
	  pcl::PointXYZRGB point = (*input_cloud)[(inliers->indices[i])];
	  bgr bgr_ = {point.b, point.g, point.r};
	  plane_color_image.at<bgr>(v, u) = bgr_;
	  plane_mask.at<uchar>(v, u) = 255;

	  // populate visibility grid
	  tf::Vector3 planePointCamera(point.x, point.y, point.z);
	  tf::Vector3 planePointWorld = transform_map_camera * planePointCamera;
	  //cv::Point2i co(-(planePointWorld.getX()-gridOrigin_.x)*gridResolution_+grid_offset.x, (planePointWorld.getY()-gridOrigin_.y)*gridResolution_+grid_offset.y);	//done: offset
	  cv::Point2i co((planePointWorld.getX()-gridOrigin_.x)*gridResolution_, (planePointWorld.getY()-gridOrigin_.y)*gridResolution_);
	  // todo: add a check whether the current point is really visible in the current image of analysis (i.e. the warped image)
	  if (visitedGridCells.find(co)==visitedGridCells.end() && co.x>=0 && co.x<grid_number_observations.cols && co.y>=0 && co.y<grid_number_observations.rows)
	  {
	    visitedGridCells.insert(co);
	    grid_number_observations.at<int>(co) = grid_number_observations.at<int>(co) + 1;
	  }

	}

	found_plane = true;
	}

	return found_plane;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dataset_create");
  ros::NodeHandle n;

  DatasetCreate dc(n);
  dc.init();
  
  ros::spin();
  return 0;
}


