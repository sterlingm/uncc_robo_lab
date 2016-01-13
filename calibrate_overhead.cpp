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
  std::cout<<"\nmins: "<<min_x<<", "<<min_y<<" maxes: "<<max_x<<", "<<max_y;

  for(uint32_t i=1;i<points.size();i++)
  {
    
    std::cout<<"\nChecking point: ("<<points.at(i).x<<", "<<points.at(i).y<<")";
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

  std::cout<<"\nmins: "<<min_x<<", "<<min_y<<" maxes: "<<max_x<<", "<<max_y;

  result.x = (min_x + max_x) / 2.;
  result.y = (min_y + max_y) / 2.;
  
  std::cout<<"\nCenter: "<<result.x<<", "<<result.y;
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

  std::cout<<"\nNumber of points on floor plane: "<<inliers->indices.size();

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(result);

  return result;
}




std::vector<std::vector<bool> > convertPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                             const double& dist_threshold)
{
  std::vector<std::vector<bool> > result(cloud->height, std::vector<bool>(cloud->width));


  int img_h = cloud->height;
  int img_w = cloud->width;

  
  //initilize with all false, no position
  for(int i = 0; i < img_h; ++i)
    for(int j = 0; j < img_w; ++j)
      result[i][j] = false;


  pcl::ModelCoefficients::Ptr       coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr            inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  // Optional
  seg.setOptimizeCoefficients (true);

  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (dist_threshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  std::cout<<"\nNumber of points on floor plane: "<<inliers->indices.size();
  
  for(int i = 0; i < inliers->indices.size(); ++i)
  {
    int index = inliers->indices[i];
    if( !isnan(cloud->points[index].x) && 
        !isnan(cloud->points[index].y) && 
        !isnan(cloud->points[index].z))
    {
      result[index/img_w][index%img_w] = true;
    }
  }


  return result;
}


/*
Find out the table region, which includes both the table plane and 
  objects on the table.
Assumption : the table region is in the middle, which occupies the most 
  space of the image.
@param table_plane : an array indicating which points belong to the table plane
@param img_h : the height of the image
@param img_w : the width of the image
@return : returns an array indicating which points belong to the table region
*/
std::vector<std::vector<bool> > detect_table_region(const std::vector<std::vector<bool> > &table_plane,
                                                  const int &img_h, const int &img_w){
std::vector<std::vector<bool> > table_region(img_h, std::vector<bool>(img_w));

  //initilize with all true
  for(int i = 0; i < img_h; ++i)
    for(int j = 0; j < img_w; ++j)
      table_region[i][j] = true;
      
  //start from the edges of the image, and find out all the regions which 
  //are outside the table plane.
  std::vector<std::vector<bool> > visited(img_h, std::vector<bool>(img_w));
  for(int i = 0; i < img_h; ++i)
    for(int j = 0; j < img_w; ++j)
      visited[i][j] = false;

  std::stack<cv::Point2i> st;
  //push all the points on the edges of the image if they are not on the tale plane
  for(int i = 0; i < img_w; ++i)
  {
    cv::Point2i push_node;
    if(!table_plane[0][i] && !visited[0][i])
    {
      push_node.y = 0;
      push_node.x = i;
      st.push(push_node);
      visited[0][i] = true;
    }
    if(!table_plane[img_h-1][i] && !visited[img_h-1][i])
    {
      push_node.y = img_h - 1;
      push_node.x = i;
      st.push(push_node);
      visited[img_h-1][i] = true;
    }
  }

  for(int i = 0; i < img_h; ++i){
    cv::Point2i push_node;
    if(!table_plane[i][0] && !visited[i][0])
    {
      push_node.y = i;
      push_node.x = 0;
      st.push(push_node);
      visited[i][0] = true;
    }
    if(!table_plane[i][img_w-1] && !visited[i][img_w-1])
    {
      push_node.y = i;
      push_node.x = img_w - 1;
      st.push(push_node);
      visited[i][img_w-1] = true;
    }
  }

  while(st.size())
  {
    cv::Point2i cur_node;
    cur_node = st.top();
    st.pop();

    table_region[cur_node.y][cur_node.x] = false;

    //push unvisited neighbors that do not belong to the talbe plane
    cv::Point2i push_node;

    push_node.y = cur_node.y - 1;
    push_node.x = cur_node.x - 1;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }

    push_node.y = cur_node.y - 1;
    push_node.x = cur_node.x;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
      
    push_node.y = cur_node.y - 1;
    push_node.x = cur_node.x + 1;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
      
    push_node.y = cur_node.y;
    push_node.x = cur_node.x - 1;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
      
    push_node.y = cur_node.y;
    push_node.x = cur_node.x + 1;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
      
    push_node.y = cur_node.y + 1;
    push_node.x = cur_node.x - 1;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
      
    push_node.y = cur_node.y + 1;
    push_node.x = cur_node.x;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
      
    push_node.y = cur_node.y + 1;
    push_node.x = cur_node.x + 1;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x])
    {
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
  }

  return table_region;
}



/*
  Segment the table region based on the connectivity to find out all 
    the objects on the table.
  @param table_region : an array indicating which points belong to the table region
  @param table_plane : an array indicating which points belong to the table plane
  @param img_h : the height of the image
  @param img_w: the width of the image
  @num_seg: returns the number of the segments of the table region
  @return: returns all the segments of the table region based on the connectivity, 0 means the background
*/
std::vector<std::vector<int> > detect_all_objects_on_table(const std::vector<std::vector<bool> > &table_region,
                                                           const std::vector<std::vector<bool> > &table_plane,
                                                           const int &img_h, const int &img_w, int &num_seg)
{
  std::vector<std::vector<int> > segment_indices(img_h, std::vector<int>(img_w));
  num_seg = 0;
  
  //initilize with all true
  for(int i = 0; i < img_h; ++i)
    for(int j = 0; j < img_w; ++j)
      segment_indices[i][j] = 0;
      
  std::vector<std::vector<bool> > visited(img_h, std::vector<bool>(img_w));
  for(int i = 0; i < img_h; ++i)
    for(int j = 0; j < img_w; ++j)
      visited[i][j] = false;
      
  for(int i = 0; i < img_h; ++i){
    for(int j = 0; j < img_w; ++j){
    if(visited[i][j] || !table_region[i][j] || table_plane[i][j])
      continue;
    
    //find out a new segment
    ++num_seg;
    std::stack<cv::Point2i> st;
    cv::Point2i fst_node;
    fst_node.y = i;
    fst_node.x = j;
    st.push(fst_node);
    visited[i][j] = true;
    
    while(st.size()){
    cv::Point2i cur_node;
    cur_node = st.top();
    st.pop();
    
    segment_indices[cur_node.y][cur_node.x] = num_seg;
      
      //push unvisited neighbors that belongs to the same segment
      cv::Point2i push_node;
  
    push_node.y = cur_node.y - 1;
    push_node.x = cur_node.x - 1;
    if(push_node.y > 0 && push_node.y < img_h && 
       push_node.x > 0 && push_node.x < img_w &&
       table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
       !visited[push_node.y][push_node.x]){
      st.push(push_node);
      visited[push_node.y][push_node.x] = true;
    }
  
      push_node.y = cur_node.y - 1;
      push_node.x = cur_node.x;
      if(push_node.y > 0 && push_node.y < img_h && 
         push_node.x > 0 && push_node.x < img_w &&
         table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
         !visited[push_node.y][push_node.x]){
        st.push(push_node);
        visited[push_node.y][push_node.x] = true;
        }
    
      push_node.y = cur_node.y - 1;
      push_node.x = cur_node.x + 1;
      if(push_node.y > 0 && push_node.y < img_h && 
         push_node.x > 0 && push_node.x < img_w &&
         table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
         !visited[push_node.y][push_node.x]){
        st.push(push_node);
        visited[push_node.y][push_node.x] = true;
        }
    
      push_node.y = cur_node.y;
      push_node.x = cur_node.x - 1;
      if(push_node.y > 0 && push_node.y < img_h && 
         push_node.x > 0 && push_node.x < img_w &&
         table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
         !visited[push_node.y][push_node.x]){
        st.push(push_node);
        visited[push_node.y][push_node.x] = true;
        }
    
      push_node.y = cur_node.y;
      push_node.x = cur_node.x + 1;
      if(push_node.y > 0 && push_node.y < img_h && 
         push_node.x > 0 && push_node.x < img_w &&
         table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
         !visited[push_node.y][push_node.x]){
        st.push(push_node);
        visited[push_node.y][push_node.x] = true;
        }
    
      push_node.y = cur_node.y + 1;
      push_node.x = cur_node.x - 1;
      if(push_node.y > 0 && push_node.y < img_h && 
         push_node.x > 0 && push_node.x < img_w &&
         table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
         !visited[push_node.y][push_node.x]){
        st.push(push_node);
        visited[push_node.y][push_node.x] = true;
        }
    
      push_node.y = cur_node.y + 1;
      push_node.x = cur_node.x;
      if(push_node.y > 0 && push_node.y < img_h && 
         push_node.x > 0 && push_node.x < img_w &&
         table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
         !visited[push_node.y][push_node.x]){
        st.push(push_node);
        visited[push_node.y][push_node.x] = true;
        }
    
      push_node.y = cur_node.y + 1;
      push_node.x = cur_node.x + 1;
      if(push_node.y > 0 && push_node.y < img_h && 
         push_node.x > 0 && push_node.x < img_w &&
         table_region[push_node.y][push_node.x] &&
       !table_plane[push_node.y][push_node.x] &&
         !visited[push_node.y][push_node.x]){
        st.push(push_node);
        visited[push_node.y][push_node.x] = true;
        }
    
    }
  }
  }  
  
  return segment_indices;
}



std::vector< std::vector<int> > getObjectIndices(const std::vector< std::vector<int> > objects)
{
  std::vector< std::vector<int> > result;

  for(int i=0;i<objects.size();i++)
  {
    for(int j=0;j<objects.at(0).size();j++)
    {
      int index = i * objects.at(0).size() + j;
      if(result.size() > objects[i][j])
      {
        result.at(objects[i][j]).push_back(index);
      }
      else
      {
        std::vector<int> ob;
        ob.push_back(index);
        result.push_back(ob);
      }
    }
  }


  // Sort by largest
  for(int i=0;i<result.size();i++)
  {
    int start = i;
    for(int j=0;j<result.size();j++)
    {
      if(result.at(j).size() < result.at(start).size())
      {
        std::vector<int> swap = result.at(i);
        result.at(i) = result.at(j);
        result.at(j) = swap;
      }
    }
  }


  for(int i=0;i<result.size();i++)
  {
    printf("\nresult[%i].size(): %i", i, (int)result[i].size());
  }

  return result;
}


const tf::Vector3 getCenter(const pcl::PointCloud<pcl::PointXYZRGB> cloud, const std::vector<int> points)
{
  /*float x_min = cloud.points[points[0]].x; 
  float x_max = cloud.points[points[0]].x; 
  float y_min = cloud.points[points[0]].y; 
  float y_max = cloud.points[points[0]].y; */

  double x_cen=0.;
  double y_cen=0.;

  for(int i=0;i<points.size();i++)
  {
    if(!isnan(cloud.points[points[i]].x) && !isnan(cloud.points[points[i]].y))
    {
      x_cen += cloud.points[points[i]].x;
      y_cen += cloud.points[points[i]].y;
    }
  }


  x_cen /= points.size();
  y_cen /= points.size();

  tf::Vector3 result(x_cen, y_cen, 0.f);
  return result;
}



const int getCenter2(const pcl::PointCloud<pcl::PointXYZRGB> cloud, const std::vector<int> points)
{
  return points.at(points.size() / 2);
}



int main(int argc, char** argv)
{

  // Get the scene point cloud
  std::string filename = "tmp/scene_1_4.pcd";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = getCloud(filename);

  std::string filename_im = "tmp/scene_1_4.png";
  writeCloud(filename, filename_im, *cloud);



  // Get the floor plane
  pcl::PointCloud<pcl::PointXYZRGB> floor_plane = getFloorPlane(cloud);

  std::string floor_pcd = "tmp/floor_plane.pcd";
  std::string floor_im = "tmp/floor_plane.jpg";
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
  writeCloud("tmp/red_square.pcd", "tmp/red_square.png", floor_plane);


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
  writeCloud("tmp/green_square.pcd", "tmp/green_square.png", floor_plane);
 

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

  std::cout<<"\np: ("<<p_cam.getX()<<", "<<p_cam.getY()<<", "<<p_cam.getZ()<<")";
  std::cout<<"\ny: ("<<y_axis.getX()<<", "<<y_axis.getY()<<", "<<y_axis.getZ()<<")";
  std::cout<<"\nx: ("<<x_axis.getX()<<", "<<x_axis.getY()<<", "<<x_axis.getZ()<<")";

  tf::Vector3 x(1,0,0);
  tf::Vector3 y(0,1,0);
  tf::Vector3 z(0,0,1);
  double x_diff = x.angle(x_axis);
  double y_diff = y.angle(y_axis);
  double z_diff = 0;//z.angle(p_cam);

  tf::Quaternion rot_cam(x_diff, y_diff, z_diff, 1);

  printf("\nx_diff: %f y_diff: %f z_diff: %f", x_diff, y_diff, z_diff);

  // Create transform we will use, FROM camera TO real-world
  tf::Transform tf_cam;
  //tf_cam.setOrigin(tf::Vector3(0, 0, 0));
  tf_cam.setOrigin(tf::Vector3(-r_cen.x, -r_cen.y, 0));
  tf_cam.setRotation(rot_cam);


  tf::Vector3 red_cen(0., 0, 0);
  tf::Vector3 trans_red_cen = tf_cam * red_cen;

  printf("\ntrans_red_cen: (%f, %f, %f)", trans_red_cen.getX(), trans_red_cen.getY(), trans_red_cen.getZ());
   


  //std::vector<uint32_t> b_square = findBlueSquare(floor_plane);
  //colorRegionWhite(floor_plane, r_square);
  //colorRegionWhite(floor_plane, b_square);
  
  //getCenterOfRegion(floor_plane, r_square);
  
  //useSegment(floor_plane);
  



/***********************************************************************************/

  filename = "tmp/scene_1_10.pcd";
  cloud = getCloud(filename);

  filename_im = "tmp/scene_1_10.png";
  writeCloud(filename, filename_im, *cloud);



  int num_seg = 50;

  std::vector<std::vector<bool> > floor_plane_vec   = convertPlane(cloud, 0.1);
  std::vector<std::vector<bool> > floor_region_vec  = detect_table_region(floor_plane_vec, cloud->height, cloud->width);


  clock_t start = clock();

  std::vector<std::vector<int> >  objects           = detect_all_objects_on_table(floor_region_vec,
                                                                                  floor_plane_vec,
                                                                                  cloud->height,
                                                                                  cloud->width,
                                                                                  num_seg);
  clock_t stop = clock();

  double elapsed = (double)(stop - start) / CLOCKS_PER_SEC;
  printf("\nElapsed time to get objects: %f\n", elapsed);
  

  //candidate colors to show the results
  unsigned char colors[100][3];
  colors[0][0] = colors[0][1] = colors[0][2] = 0;
  for(int i = 1; i < 100; ++i){
  colors[i][0] = (unsigned char)rand()%256;
  colors[i][1] = (unsigned char)rand()%256;
  colors[i][2] = (unsigned char)rand()%256;
  }
  colors[0][0] = 0;   colors[0][1] = 0;   colors[0][2] = 0;
  colors[2][0] = 0;   colors[2][1] = 255; colors[2][2] = 0; //green
  colors[3][0] = 0;   colors[3][1] = 0;   colors[3][2] = 255;  //red
  colors[4][0] = 255; colors[4][1] = 0;   colors[4][2] = 0;  //blue


  //show the segments
  cv::Mat img_bgr_segments(cloud->height, cloud->width, CV_8UC3);
  for(int i = 0; i < cloud->height; ++i)
  {
    for(int j = 0; j < cloud->width; ++j)
    {
      if(floor_region_vec[i][j] && floor_plane_vec[i][j])
      {
          img_bgr_segments.at<cv::Vec3b>(i, j)[0] = 255;
          img_bgr_segments.at<cv::Vec3b>(i, j)[1] = 0;
          img_bgr_segments.at<cv::Vec3b>(i, j)[2] = 255;
      }
      else
      {
          img_bgr_segments.at<cv::Vec3b>(i, j)[0] = colors[objects[i][j]][0];
          img_bgr_segments.at<cv::Vec3b>(i, j)[1] = colors[objects[i][j]][1];
          img_bgr_segments.at<cv::Vec3b>(i, j)[2] = colors[objects[i][j]][2];
      }
    }
  }


  //getCenterOfSegment(objects.at(1));
  printf("\nSize of objects: %i", (int)objects.size());

  cv::imwrite("img_bgr_segments.jpg", img_bgr_segments);


  std::vector<int> i_ones;
  // Find the indices of the pixels
  for(int i = 0; i < cloud->height; ++i)
  {
    for(int j = 0; j < cloud->width; ++j)
    {
      int index = i*cloud->width + j;
      //printf("\nobjects[%i][%i]: %i", i, j, objects[i][j]);
      
      if(objects[i][j] == 3)
      {
        i_ones.push_back(index);
      }
    }
  }

  std::vector< std::vector<int> > obs = getObjectIndices(objects);
  for(int i=1;i<5;i++)
  {
    //for(int j=0;j<obs[i].size();j++)
    //{
    tf::Vector3 cen = getCenter(*cloud, obs[i]);
    int k = getCenter2(*cloud, obs[i]);
    int k_row = k % cloud->width;
    int k_col = k / cloud->height;

    if(i==1)
    {
      cloud->points[k].r = 255;
      cloud->points[k].g = 255;
      cloud->points[k].b = 255;
    }

    else if(i==2)
    {
      cloud->points[k].r = 255;
      cloud->points[k].g = 0;
      cloud->points[k].b = 0;
    }

    else if(i==3)
    {
      cloud->points[k].r = 0;
      cloud->points[k].g = 255;
      cloud->points[k].b = 0;
    }

    else
    {
      cloud->points[k].r = 0;
      cloud->points[k].g = 0;
      cloud->points[k].b = 255;
    }

    tf::Vector3 k_cen(cloud->points[k].x, cloud->points[k].y, 0);
    //tf::Vector3 trans_k_cen = tf_cam * k_cen;

    printf("\nAverage Center: (%f, %f)", cen.getX(), cen.getY());
    printf("\nAverage Point center: (%f, %f)", k_cen.getX(), k_cen.getY());
    
    /*cloud->points[obs[i][j]].r = 255;
    cloud->points[obs[i][j]].g = 255;
    cloud->points[obs[i][j]].b = 255;
    }*/
  }

  
  writeCloud("test_segments.pcd", "test_segments.jpg", *cloud);

  printf("\nExiting normally\n");
  return 0;
}
