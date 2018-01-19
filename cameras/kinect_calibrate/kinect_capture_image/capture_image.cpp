#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <unistd.h>

#include "opencv2/opencv.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


 class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
         
         std::cout<<"Please input scene_index pcd_index for saving, input 0 0 to cancel"<<std::endl;
         int scene_index, pcd_index;
         std::cin>>scene_index>>pcd_index;
         char pcd_path[1024];
         sprintf(pcd_path, "tmp/scene_%d_%d.pcd", scene_index, pcd_index);
         
         pcl::io::savePCDFileASCII(pcd_path, *cloud);
	  
	     cv::Mat img(cloud->height, cloud->width, CV_8UC3);
	  for(int i = 0; i < cloud->height; ++i){
	    for(int j = 0; j < cloud->width; ++j){
		  int index = i * cloud->width + j;
		  img.at<cv::Vec3b>(i, j)[0] = cloud->points[index].b;
		  img.at<cv::Vec3b>(i, j)[1] = cloud->points[index].g;
		  img.at<cv::Vec3b>(i, j)[2] = cloud->points[index].r;
		  }
	    }
	     char rgb_path[1024];
	  
	     sprintf(rgb_path, "tmp/scene_%d_%d.png", scene_index, pcd_index);
	  
	     cv::imwrite(rgb_path, img);

	     std::cout<<"***************Saved the image*******************"<<std::endl;
     }

     void run ()
     {
       pcl::Grabber* interface = new pcl::OpenNIGrabber();

       boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

       interface->registerCallback (f);

       interface->start ();

       while (!viewer.wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       }

       interface->stop ();
     }

     pcl::visualization::CloudViewer viewer;
 };

 int main ()
 {
   SimpleOpenNIViewer v;
   v.run ();
   return 0;
 }
