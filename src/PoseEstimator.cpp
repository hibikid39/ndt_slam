#include "ndt_mapping/PoseEstimator.h"

// 初期値initPoseを与えて、ICPによりロボット位置の推定値estPoseを求める
double PoseEstimator::estimatePose(Pose2D &initPose, Pose2D &estPose) {
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

  ROS_INFO("[PoseEstimator::estimatePose] source_cloud point num %d", source_cloud->points.size());
  ROS_INFO("[PoseEstimator::estimatePose] filtered_cloud point num %d", filtered_cloud->points.size());

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
  return cost;

/*
  double min = HUGE_VAL;             // コスト最小値 初期値は大きく HUGE_VAL:巨大な値
  popt.setLimit(0.2);                // limitは外れ値の閾値[m]

  double cost = 0;                       // コスト
  double costOld = min;                  // 1つ前の値。収束判定のために使う。
  Pose2D pose = initPose;
  Pose2D poseMin = initPose;             // poseMin:最適ポーズ

  // コストの変化量が小さくなったら終了 i<100は振動対策
  for (int i=0; std::abs(costOld-cost) > thre && i<100; i++){
    if (i > 0)
      costOld = cost;
    // データ対応づけ 結果はDataAssociatorのcurLps,refLpsに格納
    double ratio = dass->findCorrespondence(curScan, pose);    // ratio:対応付けできた割合
    Pose2D newPose;
    popt.setPoints(dass->curLps, dass->refLps);               // 対応結果をコスト関数に渡す
    cost = popt.optimizePose(pose, newPose);                  // その対応づけにおいてロボット位置の最適化

//    printf("pose : pose.tx=%g, pose.ty=%g, pose.th=%g\n", pose.tx, pose.ty, pose.th);
//    printf("newPose : newPose.tx=%g, newPose.ty=%g, newPose.th=%g\n", newPose.tx, newPose.ty, newPose.th);
    pose = newPose;

    if (cost < min) {                                             // コスト最小結果を保存
      poseMin = newPose;
      min = cost;
    }
//    printf("[PoseEstimatorICP] dass->curLps.size=%lu, dass->refLps.size=%lu, ratio=%g\n", dass->curLps.size(), dass->refLps.size(), ratio);
//    printf("[PoseEstimatorICP] i=%d: cost=%g, costOld=%g\n", i, cost, costOld);
  }

  pnrate = popt.getPnrate();
  usedNum = dass->curLps.size();

  estPose = poseMin;

  ROS_INFO("[PoseEstimator::estimatePose] finalError=%g, pnrate=%g, usedNum=%d", min, pnrate, int(usedNum));
//  printf("estPose:  tx=%f, ty=%f, th=%f\n", pose.tx, pose.ty, pose.th);      // 確認用

  if (min < HUGE_VAL)
    totalError += min;                      // 誤差合計
//  printf("totalError=%f\n", totalError);    // 確認用

  return(min);

*/
}
