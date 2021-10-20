#include "ndt_mapping/PoseEstimator.h"

// 初期値initPoseを与えて、ICPによりロボット位置の推定値estPoseを求める
double PoseEstimator::estimatePose(Pose2D &initPose, Pose2D &estPose, Eigen::Matrix3d &cov) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // source_cloud data
  source_cloud->width = curScan->lps.size();
  source_cloud->height = 1;
  source_cloud->is_dense = false;
  source_cloud->points.resize(source_cloud->width * source_cloud->height);
  for (size_t i = 0; i < source_cloud->points.size(); ++i) {
    source_cloud->points[i].x = curScan->lps[i].x;
    source_cloud->points[i].y = curScan->lps[i].y;
    source_cloud->points[i].z = 0;
  }

  // target_cloud data
  target_cloud->width = refScan->lps.size();
  target_cloud->height = 1;
  target_cloud->is_dense = false;
  target_cloud->points.resize(target_cloud->width * target_cloud->height);
  for (size_t i = 0; i < target_cloud->points.size(); ++i) {
    target_cloud->points[i].x = refScan->lps[i].x;
    target_cloud->points[i].y = refScan->lps[i].y;
    target_cloud->points[i].z = 0;
  }

  // filter source_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(LeafSize, LeafSize, LeafSize);
  approximate_voxel_filter.setInputCloud(source_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);

  ROS_INFO("[PoseEstimator::estimatePose] source_cloud point num = %d", source_cloud->points.size());
  ROS_INFO("[PoseEstimator::estimatePose] filtered_cloud point num = %d", filtered_cloud->points.size());

  timer.start_timer();
  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(target_cloud);

  // Set initial alignment estimate found using robot odometry.
  Eigen::AngleAxisf init_rotation(DEG2RAD(initPose.th), Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (initPose.tx, initPose.ty, 0.0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, init_guess);
  Eigen::Matrix4f T = ndt.getFinalTransformation();

  double theta;
  if (T(0,0) > 0 && T(1,0) > 0) theta = std::asin(T(1,0));
  else if (T(0,0) > 0 && T(1,0) < 0) theta = std::asin(T(1,0));
  else if (T(0,0) < 0 && T(1,0) > 0) theta = std::acos(T(0,0));
  else theta = std::acos(T(0,0))*(-1.0);
  estPose.setPose(T(0, 3), T(1, 3), RAD2DEG(theta));

  ROS_INFO("align time");
  timer.end_timer();
  timer.print_timer();

  double cost;
  cost = ndt.getFitnessScore();
  if (ndt.hasConverged() == 0) { // 収束してなかったら失敗 スコアを大きな値に
    cost = 10000000;
  }

  double prob_aft = ndt.getTransformationProbability();
  ROS_INFO("[PoseEstimator::estimatePose] prob_aft = %f, cost=%f", prob_aft, cost);


  // hessianを計算
  Eigen::Matrix<double,6,6> hessian;
  Eigen::Matrix<double,6,1> p;
  p << T(0, 3), T(1, 3), 0.0, 0.0, 0.0, theta;
  ndt.getHessian(hessian, *output_cloud, p);
  Eigen::Matrix3d hessian3d;
  hessian3d << hessian(0,0), hessian(0,1), hessian(0,5),
               hessian(1,0), hessian(1,1), hessian(1,5),
               hessian(5,0), hessian(5,1), hessian(5,5);
  hessian3d *= -1;
//  std::cout << "H = " << std::endl;
//  std::cout << hessian3d << std::endl;
  cov = hessian3d.inverse() * coeNDTCov;
//  std::cout << "cov = " << std::endl;
//  std::cout << cov << std::endl;

  return cost;
}
