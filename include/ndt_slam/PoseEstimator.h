#ifndef POSEESTIMATOR_H_
#define POSEESTIMATOR_H_

#include <ros/ros.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
//#include "PoseOptimizer.h"
//#include "DataAssociator.h"
#include "Timer.h"

namespace pcl{
  template<typename PointSource, typename PointTarget>
  class NDT : public NormalDistributionsTransform<PointSource, PointTarget>
  {
  public:
    typedef typename NormalDistributionsTransform<PointSource, PointTarget>::PointCloudSource PointCloudSource;
    void getHessian (Eigen::Matrix<double, 6, 6> &hessian,
                      PointCloudSource &trans_cloud,
                      Eigen::Matrix<double, 6, 1> &p) {
      NormalDistributionsTransform<PointSource, PointTarget>::computeHessian(hessian, trans_cloud, p);
    }
  };
}


//////

class PoseEstimator{
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud; // 現在スキャン
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud; // 参照マップ

  const Scan2D *curScan;       // 現在スキャン ポインタ
  const Scan2D *refScan;
  double coeNDTCov;

  double TransformationEpsilon;
  double StepSize;
  double Resolution;
  int MaximumIterations;

  double LeafSize; // フィルタサイズ

//  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  pcl::NDT<pcl::PointXYZ, pcl::PointXYZ> ndt;

  Timer timer;

public:
  double totalError;           // 誤差合計
//  DataAssociator *dass;        // データ対応づけクラス

public:

  PoseEstimator() : coeNDTCov(1.0),
  TransformationEpsilon(0.01), StepSize(0.1), Resolution(1.0), MaximumIterations(35), LeafSize(0.1) {
    ros::param::get("coeNDTCov", coeNDTCov);
    ros::param::get("TransformationEpsilon", TransformationEpsilon);
    ros::param::get("StepSize", StepSize);
    ros::param::get("Resolution", Resolution);
    ros::param::get("MaximumIterations", MaximumIterations);
    ros::param::get("LeafSize", LeafSize);

    source_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    target_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(TransformationEpsilon);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(StepSize);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(Resolution);
    // Setting max number of registration iterations.
    ndt.setMaximumIterations(MaximumIterations);
  }

  ~PoseEstimator() {
  }

///////

  void setScanPair(const Scan2D *curScan, pcl::PointCloud<pcl::PointXYZ>::Ptr refScan) {
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
    
    target_cloud = refScan;
  }

  void setScanPair(const Scan2D *curScan, const Scan2D *refScan) {
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
  }

////////////////////////////////////////////////////////////////////////////////

  double estimatePose(Pose2D &initPose, Pose2D &estPose, Eigen::Matrix3d &cov);
};

#endif
