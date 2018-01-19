#include "transformation_estimation_fixedCam_chessboard.h"

//for moved cameras, need pcd1 pcd2 load from pcd files 
//for camera and chessboard, pcd1 from pcd file  and pcd writen manually

int main ()
{
	char pcd1_path[1024];

	char keypoint_pairs_path[1024];
	char pcd1_transform_path[1024];
    char pcd2_transform_path[1024];
    char optToRobot_transform_path[1024];
    
    sprintf(pcd1_path, "/home/huitan/codes/cam_calib/transformation_estimation_fixedCam_chessboard/scene_1_1.pcd");

    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcd1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd1_path, *pcd1);

    //chessboard points
	sprintf(keypoint_pairs_path, "/home/huitan/codes/cam_calib/transformation_estimation_fixedCam_chessboard/scene_1_1_calib/scene_1_1_calib_2d1.txt");
	
	std::ifstream keypoint_pairs_cin;
    keypoint_pairs_cin.open(keypoint_pairs_path);
    if(!keypoint_pairs_cin.is_open()){
	  std::cout<<"ATTENTION: Failed to Open Keypoint Pairs Files!\n"<<std::endl;
	}
	
	int num_keypoint_pairs;
    keypoint_pairs_cin>>num_keypoint_pairs;
    
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_tgt(new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	match_src->points.clear();
	match_src->height = 1;
	match_tgt->points.clear();
	match_tgt->height = 1;
	match_tgt->points.resize(48);
	
    //read the valid key point pairs from the file
    for(int i = 0; i < num_keypoint_pairs; ++i){
	  double dcol1, drow1;
	  keypoint_pairs_cin>>dcol1>>drow1;
	  
	  int col1, row1;
	  col1 = (int)(dcol1 + 0.5);
	  row1 = (int)(drow1 + 0.5);


	  int pcd1_index;
	  pcd1_index = row1 * pcd1->width + col1;
	 
	  
	  if(!isnan(pcd1->points[pcd1_index].z)){
		match_src->points.push_back(pcd1->points[pcd1_index]);   //w.r.t camera 
		
		int Y_ind = i/8;
	    match_tgt->points[i].x = 0.028*(i%8);        //w.r.t chessboard  
	    match_tgt->points[i].y = 0.028*(5-Y_ind);    //已经是取址 需要resize
	    match_tgt->points[i].z = 0.0;
	  
	  }
	}
	
	
	keypoint_pairs_cin.close();
	
	match_src->width = match_src->points.size();
	match_tgt->width = match_tgt->points.size();
      
 	Eigen::Matrix4f transform_srcTotgt;
	RANSAC_optimal_rigid_transformation(match_src, match_tgt, transform_srcTotgt); 
	std::cout << "**********transformation matrix from camera to chessboard:\n" << transform_srcTotgt << std::endl;
	
	
	
	//read the transformation matrix file from chessboard to robot world frame
    sprintf(pcd1_transform_path, "/home/huitan/codes/cam_calib/transformation_estimation_fixedCam_chessboard/chessboard_to_robotWrold_tf.txt");
    std::ifstream pcd1_transform_cin;
    pcd1_transform_cin.open(pcd1_transform_path);
	if(!pcd1_transform_cin.is_open()){
	  std::cout<<"ATTENTION: Fail to Open Transformation File!\n"<<std::endl;
	}
    Eigen::Matrix4f pcd1_transform;
    for(int i = 0; i < 4; ++i)
      for(int j = 0; j < 4; ++j)
        pcd1_transform_cin>>pcd1_transform(i, j);
    pcd1_transform_cin.close();
	printf("**********transformation matrix from chessboard to robot world frame:\n");
    std::cout<<pcd1_transform<<std::endl;
	
	//calculate transformation matrix from camera to robot world frame
	Eigen::Matrix4f pcd2_transform;
	pcd2_transform = pcd1_transform * transform_srcTotgt;
	printf("**********transformation matrix from camera to robot world frame:\n");
	std::cout<<pcd2_transform<<std::endl;
	
	//save the transformation matrix from camera to robot world frame
    sprintf(pcd2_transform_path, "/home/huitan/codes/cam_calib/transformation_estimation_fixedCam_chessboard/camera_to_robotWorld_tf.txt");
    freopen(pcd2_transform_path, "w", stdout);
    std::cout<<pcd2_transform<<std::endl;
    freopen("/dev/tty", "w", stdout);
    //calculate transformation matrix from camera optical frame to robot world frame (optical frame is the frame one should use)
    Eigen::Matrix4f tf_camLinkToOpt;
    tf_camLinkToOpt << 0, -1, 0, -0.02,
                       0, 0, -1, 0,
                       1, 0, 0, 0,
                       0, 0, 0, 1;

    Eigen::Matrix4f camToRobot;
    camToRobot = pcd2_transform*tf_camLinkToOpt;
    std::cout<< "tf camera optical to robot world frame:\n" << camToRobot << endl;
    //save as
    sprintf(optToRobot_transform_path, "/home/huitan/codes/cam_calib/transformation_estimation_fixedCam_chessboard/cameraOptical_to_robotWorld_tf.txt");
    freopen(optToRobot_transform_path, "w", stdout);
    std::cout<<camToRobot<<std::endl;
    freopen("/dev/tty", "w", stdout);
    
    //determine yaw(aplha) pitch(beta) roll(gama) 
    double alpha, beta, gama; 
    beta = atan2(-camToRobot(2,0), sqrt(camToRobot(0,0)*camToRobot(0,0)+camToRobot(1,0)*camToRobot(1,0)));
    alpha = atan2(camToRobot(1,0)/cos(beta),camToRobot(0,0)/cos(beta));    
    gama = atan2(camToRobot(2,1)/cos(beta),camToRobot(2,2)/cos(beta));
    
    std::cout << "yaw(aplha): " << alpha << " pitch(beta): " << beta << " roll(gama): " << gama << " in radian " << std::endl;
    
    
    //*******TEST*******object position vector from camera frame to chessboard frame
    int pcd_index = 95*pcd1->width+505;   //95 is pixel row coordinate 505 is pixel column coordinate
    Eigen::Vector4f u(pcd1->points[pcd_index].x,
                      pcd1->points[pcd_index].y,
                      pcd1->points[pcd_index].z,
                      1.0);  //u is object position vector in camera frame (in point cloud)
    Eigen::Vector4f v = transform_srcTotgt*u;
    std::cout << "[TEST] v is object position vector in chessboard frame:\n " << v << std::endl;
    std::cout << "\n" << pcd1->points[pcd_index].x << " " << pcd1->points[pcd_index].y << " " << pcd1->points[pcd_index].z << std::endl;
}



//compute the optimal transformation from the source to the target
//based on the coordinate information of their keypoint pairs
void search_optimal_rigid_transformation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &match_src,
                                         const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &match_tgt,
                                         Eigen::Matrix4f &transform_optimal){
  float min_dist = 1.0f;
  
  int num_keypoint_pairs = match_src->points.size();
  Eigen::MatrixXf src_keypoints = Eigen::MatrixXf::Zero(4, num_keypoint_pairs);
  Eigen::MatrixXf tgt_keypoints = Eigen::MatrixXf::Zero(4, num_keypoint_pairs);
  
  for(int i = 0; i < num_keypoint_pairs; ++i){
	src_keypoints(0, i) = match_src->points[i].x;
	src_keypoints(1, i) = match_src->points[i].y;
	src_keypoints(2, i) = match_src->points[i].z;
	src_keypoints(3, i) = 1.0;
	tgt_keypoints(0, i) = match_tgt->points[i].x;
	tgt_keypoints(1, i) = match_tgt->points[i].y;
	tgt_keypoints(2, i) = match_tgt->points[i].z;
	tgt_keypoints(3, i) = 1.0;
  }
  
  int num_repeat = 1000;
  int num_pairs_in_set = (int)(0.90 * num_keypoint_pairs);
  
  while(num_repeat--){
	  
	//choose a small set of key point pairs randomly
	Eigen::VectorXi keypoints_id = Eigen::VectorXi::Zero(num_keypoint_pairs);
	for(int i = 0; i < num_keypoint_pairs; ++i)
	  keypoints_id(i) = i;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_src_subset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_tgt_subset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    match_src_subset->points.clear();
    match_tgt_subset->points.clear();
    match_src_subset->height = 1;
    match_tgt_subset->height = 1;

	for(int index = num_keypoint_pairs-1; index >= num_keypoint_pairs-num_pairs_in_set; --index){
	  int index_choose = rand() % (index + 1);
	  int id_choose;
	  
	  id_choose = keypoints_id(index_choose);
	  keypoints_id(index_choose) = keypoints_id(index);

	  //copy the chosen keypoints to the vector
	  match_src_subset->points.push_back(match_src->points[id_choose]);
	  match_tgt_subset->points.push_back(match_tgt->points[id_choose]);
	}
	
	match_src_subset->width = match_src_subset->points.size();
	match_tgt_subset->width = match_tgt_subset->points.size();
	
	Eigen::Matrix<float, 4, 4> transform_local;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA,pcl::PointXYZRGBA> tf_est;
    tf_est.estimateRigidTransformation(*match_src_subset, *match_tgt_subset, transform_local);
    
	Eigen::MatrixXf src_keypoints_transformed = transform_local * src_keypoints;
	float local_dist = 0.0;
	for(int i = 0; i < num_keypoint_pairs; ++i)
	  for(int j = 0; j < 3; ++j)
	    local_dist += (tgt_keypoints(j, i) - src_keypoints_transformed(j, i)) * 
	                  (tgt_keypoints(j, i) - src_keypoints_transformed(j, i));                  
	local_dist = sqrtf(local_dist/num_keypoint_pairs);
	                  
	if(local_dist < min_dist){
	  min_dist = local_dist;
	  transform_optimal = transform_local;
	}

  }
  
  printf("**********min_dist : %3.9f\n", min_dist);
  std::cout<<"**********transform_optimal :"<<std::endl<<transform_optimal<<std::endl;			
}

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
                                         Eigen::Matrix4f &transform_optimal){
  float best_error = 1.0f;
  int num_best_consensus = 0;
  
  int num_keypoint_pairs = match_src->points.size();
  Eigen::MatrixXf src_keypoints = Eigen::MatrixXf::Zero(4, num_keypoint_pairs);
  Eigen::MatrixXf tgt_keypoints = Eigen::MatrixXf::Zero(4, num_keypoint_pairs);
  
  for(int i = 0; i < num_keypoint_pairs; ++i){
	src_keypoints(0, i) = match_src->points[i].x;
	src_keypoints(1, i) = match_src->points[i].y;
	src_keypoints(2, i) = match_src->points[i].z;
	src_keypoints(3, i) = 1.0;
	tgt_keypoints(0, i) = match_tgt->points[i].x;
	tgt_keypoints(1, i) = match_tgt->points[i].y;
	tgt_keypoints(2, i) = match_tgt->points[i].z;
	tgt_keypoints(3, i) = 1.0;
  }
  
  int num_repeat = 1000;
  int num_pairs_in_set = (int)(0.8 * num_keypoint_pairs);
  
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA,pcl::PointXYZRGBA> tf_est;
  
  while(num_repeat--){

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_src_subset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_tgt_subset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    match_src_subset->points.clear();
    match_tgt_subset->points.clear();
    match_src_subset->height = 1;
    match_tgt_subset->height = 1;
    
    //maybe_inliers := n randomly selected values from data	  
	//choose a small set of key point pairs randomly
	Eigen::VectorXi keypoints_id = Eigen::VectorXi::Zero(num_keypoint_pairs);
	for(int i = 0; i < num_keypoint_pairs; ++i)
	  keypoints_id(i) = i;

    for(int index = num_keypoint_pairs-1; index >= num_keypoint_pairs-num_pairs_in_set; --index){
	  int index_choose = rand() % (index + 1);
	  int id_choose;
	  
	  id_choose = keypoints_id(index_choose);
	  keypoints_id(index_choose) = keypoints_id(index);

	  //copy the chosen keypoints to the vector
	  match_src_subset->points.push_back(match_src->points[id_choose]);
	  match_tgt_subset->points.push_back(match_tgt->points[id_choose]);
	}
		
	match_src_subset->width = match_src_subset->points.size();
	match_tgt_subset->width = match_tgt_subset->points.size();
	
	//maybe_model := model parameters fitted to maybe_inliers
    Eigen::Matrix<float, 4, 4> maybe_transform;
    tf_est.estimateRigidTransformation(*match_src_subset, *match_tgt_subset, maybe_transform);
    Eigen::MatrixXf src_keypoints_transformed_maybe = maybe_transform * src_keypoints;
    
    /*
    for every point in data not in maybe_inliers 
        if point fits maybe_model with an error smaller than t
            add point to consensus_set
    to implement for convenience, we consider all the points again
    */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_src_consensus(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr match_tgt_consensus(new pcl::PointCloud<pcl::PointXYZRGBA>);        
    
    match_src_consensus->points.clear();
    match_tgt_consensus->points.clear();
    match_src_consensus->height = 1;
    match_tgt_consensus->height = 1;   
    
    int num_fit_maybe = 0;
    float fit_threshold_square = 0.0001f;
    
    for(int i = 0; i < num_keypoint_pairs; ++i){				
	  float dist = 0.0;
	  for(int k = 0; k < 3; ++k){
	    dist += (tgt_keypoints(k, i) - src_keypoints_transformed_maybe(k, i)) *
	            (tgt_keypoints(k, i) - src_keypoints_transformed_maybe(k, i));
	  }
	  
	  if(dist < fit_threshold_square){
		++num_fit_maybe;
		match_src_consensus->points.push_back(match_src->points[i]);
		match_tgt_consensus->points.push_back(match_tgt->points[i]);
	  }	  
	}
	
	match_src_consensus->width = match_src_consensus->points.size();
	match_tgt_consensus->width = match_tgt_consensus->points.size();

    /*7
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
    */
    int num_fit_threshold = (int)(0.8 * num_keypoint_pairs);
    if(num_fit_maybe > num_fit_threshold){		
	  Eigen::Matrix<float, 4, 4> good_transform;
	  tf_est.estimateRigidTransformation(*match_src_consensus, *match_tgt_consensus, good_transform);
      Eigen::MatrixXf src_keypoints_transformed_good = good_transform * src_keypoints;
      
      float local_error = 0.0;
      int local_num_valid = 0;
      for(int i = 0; i < num_keypoint_pairs; ++i){
		float dist = 0.0f;
        for(int k = 0; k < 3; ++k){
	      dist += (tgt_keypoints(k, i) - src_keypoints_transformed_good(k, i)) *
	              (tgt_keypoints(k, i) - src_keypoints_transformed_good(k, i)); 
		}
		if(dist < fit_threshold_square){
		  ++local_num_valid;
		  local_error += sqrtf(dist);
		}
	  }
	  local_error = local_error/local_num_valid;
	  
	  if(local_error < best_error){
		best_error = local_error;
		num_best_consensus = local_num_valid;
		transform_optimal = good_transform;
	  }  	
	}

  }
  
  printf("**********best_error : %3.9f\n", best_error);
  printf("**********num_best_consensus : %d\n", num_best_consensus);
  std::cout<<"**********transform_optimal :"<<std::endl<<transform_optimal<<std::endl;	
}
