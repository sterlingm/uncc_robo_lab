
/*
Definitions of functions used for capturing images from Kinect
*/

#ifndef CAPTURE_IMAGE_H
#define CAPTURE_IMAGE_H

#include <iostream>
#include <unistd.h>

#include "opencv2/opencv.hpp"
#include "pcl/io/openni_grabber.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

class SimpleOpenNIViewer{
 public:
  //SimpleOpenNIViewer(): viewer("PCL OpenNI Viewer"){}
  SimpleOpenNIViewer(){};

  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);

  void run();

  //pcl::visualization::CloudViewer viewer;
};

void capture_image();

void crop_image();

#endif       //CAPTURE_IMAGE_H
