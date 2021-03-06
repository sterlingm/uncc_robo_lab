#include "calibrate.h"
#include <pcl/point_types.h>
#include <typeinfo>
#include <string>
#include <time.h>





pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(const std::string filename, bool print=false)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>); 

  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *result) == -1)
  {
    PCL_ERROR("Couldn't read file");
  }
  
  if(print)
  { 
    std::cout<<"Loaded "
      <<result->width * result->height
      <<" data points from "<<filename<<" with the following fields: "<<std::endl;

    std::cin.get();

    for(size_t i=0;i<result->points.size();i++)
    {
      std::cout << "    " <<result->points[i].x
                << " "    <<result->points[i].y
                << " "    <<result->points[i].z<<std::endl;
    }
  }

  

  return result;
}



void writeCloud(const std::string filename_pcd, const std::string filename_image, const pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  pcl::io::savePCDFileASCII(filename_pcd, cloud);

  std::cout<<"\nCloud height: "<<cloud.height;
  std::cout<<"\nCloud width: "<<cloud.width;
    
  cv::Mat img(cloud.height, cloud.width, CV_8UC3);
  for(int i = 0; i < cloud.height; ++i) 
  {
    for(int j = 0; j < cloud.width; ++j) 
    {
      int index = i * cloud.width + j;

      img.at<cv::Vec3b>(i, j)[0] = cloud.points[index].b;
      img.at<cv::Vec3b>(i, j)[1] = cloud.points[index].g;
      img.at<cv::Vec3b>(i, j)[2] = cloud.points[index].r;
    }
  }

  cv::imwrite(filename_image, img);
}


const std::vector<pcl::PointIndices> getRGBClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::IndicesPtr indices (new std::vector <int>);  
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(1.0,4.0);
  pass.filter(*indices);


  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(cloud_ptr);
  reg.setIndices(indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (5);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(100);

  std::cout<<"\nExtracting clusters...\n";
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  return clusters;
}



const std::vector<pcl::PointIndices> getRGBClusters(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = cloud.makeShared();
  return getRGBClusters(cloud_ptr);
}


/**
 *
 */
const pcl::PointIndices findRedSquare(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::vector<pcl::PointIndices> rgb_clusters)
{
  pcl::PointIndices result;
  std::vector<pcl::PointIndices> red_clusters;
 
  // Go through the clusters, find all the red ones, return the largest one
  for(int i=0;i<rgb_clusters.size();i++)
  {
    std::cout<<"\nRegion size: "<<rgb_clusters.at(i).indices.size();
    std::cout<<"\nRegion RGB: "<<(int)cloud.points[rgb_clusters.at(i).indices.at(0)].r<<
            " "<<(int)cloud.points[rgb_clusters.at(i).indices.at(0)].g<<
            " "<<(int)cloud.points[rgb_clusters.at(i).indices.at(0)].b<<"\n";

    // If this region is red, push it onto the vector of red rgb_clusters
    if( (int)cloud.points[rgb_clusters.at(i).indices.at(0)].r > 75 && 
        (int)cloud.points[rgb_clusters.at(i).indices.at(0)].g < 50  && 
        (int)cloud.points[rgb_clusters.at(i).indices.at(0)].b < 50)
    {
      red_clusters.push_back(rgb_clusters.at(i)); 
    }
  } // end for

  // Find the largest red cluster
  uint8_t i_red_square = 0;
  for(int i=1;i<red_clusters.size();i++)
  {
    if(red_clusters.at(i).indices.size() > red_clusters.at(i_red_square).indices.size())
    {
      i_red_square = i;
    }
  } // end for

  // Return the largest cluster 
  result = red_clusters.at(i_red_square);

  return result;
}

/**
 * 
 */
const pcl::PointIndices findRedSquare(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  std::vector<pcl::PointIndices> clusters = getRGBClusters(cloud);
  return findRedSquare(cloud, clusters);
}





/**
 *
 */
const pcl::PointIndices findGreenSquare(pcl::PointCloud<pcl::PointXYZRGB> cloud, std::vector<pcl::PointIndices> rgb_clusters)
{
  pcl::PointIndices result;
  std::vector<pcl::PointIndices> green_clusters;
 
  // Go through the clusters, find all the green ones, return the largest one
  for(int i=0;i<rgb_clusters.size();i++)
  {
    std::cout<<"\nRegion size: "<<rgb_clusters.at(i).indices.size();
    std::cout<<"\nRegion RGB: "<<(int)cloud.points[rgb_clusters.at(i).indices.at(0)].r<<
            " "<<(int)cloud.points[rgb_clusters.at(i).indices.at(0)].g<<
            " "<<(int)cloud.points[rgb_clusters.at(i).indices.at(0)].b;

    // If this region is green, push it onto the vector of green rgb_clusters
    if( (int)cloud.points[rgb_clusters.at(i).indices.at(0)].r < 75  && 
        (int)cloud.points[rgb_clusters.at(i).indices.at(0)].g > 60  && 
        (int)cloud.points[rgb_clusters.at(i).indices.at(0)].b < 57)
    {
      green_clusters.push_back(rgb_clusters.at(i)); 
    }
  } // end for

  // Find the largest green cluster
  uint8_t i_green_square = 0;
  for(int i=1;i<green_clusters.size();i++)
  {
    if(green_clusters.at(i).indices.size() > green_clusters.at(i_green_square).indices.size())
    {
      i_green_square = i;
    }
  } // end for

  // Return the largest cluster 
  result = green_clusters.at(i_green_square);


  std::cout<<"\nReturning green square\n";
  return result;
}

/**
 * 
 */
const pcl::PointIndices findGreenSquare(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  std::vector<pcl::PointIndices> clusters = getRGBClusters(cloud);
  return findGreenSquare(cloud, clusters);
}




/**
 * \param cloud 
 * \param region_points indices of the region
 */
pcl::PointXYZRGB getCenterOfSquare(std::vector<pcl::PointXYZRGB> points)
{
  pcl::PointXYZRGB result = points.at(0);

  float min_x = points.at(0).x;
  float min_y = points.at(0).y;
  float max_x = min_x;
  float max_y = min_y;

  for(uint32_t i=1;i<points.size();i++)
  {
    
    if( points.at(i).x < min_x) 
    {
      min_x = points.at(i).x;
    }
    if( points.at(i).x > max_x) 
    {
      max_x = points.at(i).x;
    }
    if( points.at(i).y < min_y) 
    {
      min_y = points.at(i).y;
    }
    if( points.at(i).y > max_y) 
    {
      max_y = points.at(i).y;
    }
  } // end for 

  result.x = (min_x + max_x) / 2.;
  result.y = (min_y + max_y) / 2.;
  
  return result;
}


pcl::PointCloud<pcl::PointXYZRGB> getFloorPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::ModelCoefficients::Ptr       coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr            inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGB> result;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(result);

  return result;
}




int main(int argc, char** argv)
{
  //ros::init(argc, argv, "calibrate_overhead");
  //ros::NodeHandle handle;

  // Get the scene point cloud
  std::string filename = "../tmp/scene_1_1.pcd";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = getCloud(filename);

  std::string filename_im = "../tmp/scene_1_1.png";
  writeCloud(filename, filename_im, *cloud);



  // Get the floor plane
  pcl::PointCloud<pcl::PointXYZRGB> floor_plane = getFloorPlane(cloud);

  std::string floor_pcd = "../tmp/floor_plane.pcd";
  std::string floor_im = "../tmp/floor_plane.jpg";
  writeCloud(floor_pcd, floor_im, floor_plane);



  // Get the RGB clusters from floor plane
  std::vector<pcl::PointIndices> rgb_clusters = getRGBClusters(floor_plane);


  // Get the red square
  pcl::PointIndices i_r_square = findRedSquare(floor_plane, rgb_clusters);
  std::vector<pcl::PointXYZRGB> r_square;

  // Color it (ONLY FOR VIEWING PURPOSES)
  for(int i=0;i<i_r_square.indices.size();i++)
  {
    floor_plane.points[i_r_square.indices.at(i)].r = 255;
    floor_plane.points[i_r_square.indices.at(i)].g = 255;
    floor_plane.points[i_r_square.indices.at(i)].b = 255;
    r_square.push_back(floor_plane.points[i_r_square.indices.at(i)]);
  }
  writeCloud("../tmp/red_square.pcd", "../tmp/red_square.png", floor_plane);


  // Get the green square
  pcl::PointIndices i_g_square = findGreenSquare(floor_plane, rgb_clusters);
  std::vector<pcl::PointXYZRGB> g_square;

  // Color it (ONLY FOR VIEWING PURPOSES)
  for(int i=0;i<i_g_square.indices.size();i++)
  {
    floor_plane.points[i_g_square.indices.at(i)].r = 255;
    floor_plane.points[i_g_square.indices.at(i)].g = 255;
    floor_plane.points[i_g_square.indices.at(i)].b = 255;
    g_square.push_back(floor_plane.points[i_g_square.indices.at(i)]);
  }
  writeCloud("../tmp/green_square.pcd", "../tmp/green_square.png", floor_plane);
 

  pcl::PointXYZRGB r_cen = getCenterOfSquare(r_square);
  pcl::PointXYZRGB g_cen = getCenterOfSquare(g_square);
  
  std::cout<<"\nRed square center: "<<r_cen.x<<", "<<r_cen.y<<", "<<r_cen.z;
  std::cout<<"\nGreen square center: "<<g_cen.x<<", "<<g_cen.y<<", "<<g_cen.z;


  // Real-life transformation
  tf::Transform tf_real;
  tf::Vector3 p(0,0,0);
  tf::Quaternion rot = tf::createQuaternionFromRPY(0,0,0);
  tf_real.setOrigin(p);
  tf_real.setRotation(rot);

  // Red square is (0,0)
  // Vector is negative of difference 
  // p_cam is also the difference in the square's y value
  tf::Vector3 p_cam( g_cen.x - r_cen.x, g_cen.y - r_cen.y, 0);
  tf::Vector3 y_axis = p_cam;
  tf::Vector3 p_cam_normed = p_cam.normalized();
  tf::Vector3 z_axis(0, 0, 1);
  tf::Vector3 x_axis = p_cam_normed.cross(z_axis);

  tf::Vector3 x(1,0,0);
  tf::Vector3 y(0,1,0);
  tf::Vector3 z(0,0,1);
  double x_diff = x.angle(x_axis);
  double y_diff = y.angle(y_axis);
  double z_diff = 0;

  tf::Quaternion rot_cam(x_diff, y_diff, z_diff, 1);


  /*std::cout<<"\np: ("<<p_cam.getX()<<", "<<p_cam.getY()<<", "<<p_cam.getZ()<<")";
  std::cout<<"\ny: ("<<y_axis.getX()<<", "<<y_axis.getY()<<", "<<y_axis.getZ()<<")";
  std::cout<<"\nx: ("<<x_axis.getX()<<", "<<x_axis.getY()<<", "<<x_axis.getZ()<<")";*/
  //printf("\nx_diff: %f y_diff: %f z_diff: %f", x_diff, y_diff, z_diff);


  // Create transform we will use, FROM camera TO real-world
  tf::Transform tf_cam;
  tf_cam.setOrigin(tf::Vector3(-r_cen.x, -r_cen.y, 0));
  tf_cam.setRotation(rot_cam);


  //static tf::TransformBroadcaster br;
  //br.sendTransform(tf::StampedTransform(tf_cam, ros::Time::now(), "world", "cam_overhead"));
  

  printf("\nExiting normally\n");
  return 0;
}
