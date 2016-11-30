
#include "capture-image.h"

pcl::visualization::CloudViewer viewer_global("results");

void SimpleOpenNIViewer::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){
  if(!viewer_global.wasStopped()){
  std::cout<<"Image@ Width: "<<cloud->width<<" Height: "<<cloud->height<<" Is_dense: "<<cloud->is_dense<<std::endl;
    viewer_global.showCloud(cloud);

#define CAPTURE_AND_SAVE
#ifdef CAPTURE_AND_SAVE
  char choice;
  std::cout<<"Do you want to save this image?(y/n)"<<std::endl;

//#define CAPTURE_OBJECTS
#define CAPTURE_SCENES	
#ifdef CAPTURE_OBJECTS
  std::cout<<"Please input pose_index pcd_index for saving, input 0 0 to cancel"<<std::endl;
#endif
#ifdef CAPTURE_SCENES
  std::cout<<"Please input scene_index pcd_index for saving, input 0 0 to cancel"<<std::endl;
#endif

#ifdef CAPTURE_OBJECTS	
  int pose_index, pcd_index;	
  std::cin>>pose_index>>pcd_index;
  std::string obj_name = "tray_1";
#endif
#ifdef CAPTURE_SCENES
  int scene_index, pcd_index;	
    std::cin>>scene_index>>pcd_index;
#endif
  
#ifdef CAPTURE_OBJECTS
  if(pose_index && pcd_index){
#endif
#ifdef CAPTURE_SCENES
    if(scene_index && pcd_index){
#endif
    char pcd_path[1024];
#ifdef CAPTURE_OBJECTS	
    sprintf(pcd_path, "tmp/%s_%d_%d.pcd", obj_name.c_str(), pose_index, pcd_index);
#endif
#ifdef CAPTURE_SCENES
    sprintf(pcd_path, "tmp/scene_%d_%d.pcd", scene_index, pcd_index);
#endif
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
#ifdef CAPTURE_OBJECTS	
    sprintf(rgb_path, "tmp/%s_%d_%d.png", obj_name.c_str(), pose_index, pcd_index);
#endif	  
#ifdef CAPTURE_SCENES
    sprintf(rgb_path, "tmp/scene_%d_%d.png", scene_index, pcd_index);
#endif
    cv::imwrite(rgb_path, img);

    std::cout<<"***************Saved the image*******************"<<std::endl;
  }
#endif

  }
}

void SimpleOpenNIViewer::run(){
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
  boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
  
  interface->registerCallback(f);
  interface->start();

  while(!viewer_global.wasStopped()){
  boost::this_thread::sleep(boost::posix_time::seconds(1));
  }
  
  interface->stop();
}

int main() { 
  SimpleOpenNIViewer v;
  v.run();
}



