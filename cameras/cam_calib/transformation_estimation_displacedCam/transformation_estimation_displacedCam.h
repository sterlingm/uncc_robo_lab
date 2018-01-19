/*
Definitions of functions used for transformation estimation between images of 
* different viewpoints and poses
*/

#ifndef TRANSFORMATION_ESTIMATION_H
#define TRANSFORMATION_ESTIMATION_H

#include <iostream>
#include <fstream>
#include <unistd.h>

#include <math.h>

#include "opencv2/opencv.hpp"

#include "pcl/visualization/cloud_viewer.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/transforms.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/registration/transformation_estimation_svd.h"

#include "Eigen/Dense"
#include "Eigen/SVD"

//compute the relative transformation matrix for two images of different views
//from two different poses,
//usually, pcd1 is used as a base coordinate system,
//the transformation matrix of pcd2 view is estimated based on pcd1 view,
//asift key points are extracted for each image and matched between them for 
//transformation estimation.
//pcd1 is the target and pcd2 is the source,
//transformation matrix is used to transform all the points in pcd2 to pcd1


//////////////////////////////////////////////////////void estimate_rigid_transformation_asift_keypoint_pairs(const int &pcd_choice);


//compute the optimal transformation from the source to the target
//based on the coordinate information of their keypoint pairs
void search_optimal_rigid_transformation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &match_src,
                                         const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &match_tgt,
                                         Eigen::Matrix4f &transform_optimal);
                                         
//compute the optimal transformation from the source to the target
//based on the coordinate information of their keypoint pairs
//RANSAC
/************************RANSAC Algorithm**************************************
From: http://en.wikipedia.org/wiki/RANSAC
input:
    data - a set of observations
    model - a model that can be fitted to data 
    n - the minimum number of data required to fit the model
    k - the number of iterations performed by the algorithm
    t - a threshold value for determining when a datum fits a model
    d - the number of close data values required to assert that a model fits well to data
output:
    best_model - model parameters which best fit the data (or nil if no good model is found)
    best_consensus_set - data points from which this model has been estimated
    best_error - the error of this model relative to the data 

iterations := 0
best_model := nil
best_consensus_set := nil
best_error := infinity
while iterations < k 
    maybe_inliers := n randomly selected values from data
    maybe_model := model parameters fitted to maybe_inliers
    consensus_set := maybe_inliers

    for every point in data not in maybe_inliers 
        if point fits maybe_model with an error smaller than t
            add point to consensus_set
    
    if the number of elements in consensus_set is > d 
        (this implies that we may have found a good model,
        now test how good it is)
        this_model := model parameters fitted to all points in consensus_set
        this_error := a measure of how well this_model fits these points
        if this_error < best_error
            (we have found a model which is better than any of the previous ones,
            keep it until a better one is found)
            best_model := this_model
            best_consensus_set := consensus_set
            best_error := this_error
     
    increment iterations

return best_model, best_consensus_set, best_error
************************RANSAC Ends*******************************************/
void RANSAC_optimal_rigid_transformation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &match_src,
                                         const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &match_tgt,
                                         Eigen::Matrix4f &transform_optimal);

#endif    //TRANSFORMATION_ESTIMATION_H
