#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud(const std::string filename, bool print=false)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>); 

  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, *result) == -1)
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



int main(int argc, char** argv)
{
  std::string filename = argv[1];

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = getCloud(filename);
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }

  return 0;
}
