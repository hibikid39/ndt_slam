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
  const Scan2D *curScan;       // 現在スキャン ポインタ
  const Scan2D *refScan;
  double coeNDTCov;

  double TransformationEpsilon;
  double StepSize;
  double Resolution;
  int MaximumIterations;

  double LeafSize; // フィルタサイズ

//  PoseOptimizer popt;         // 最適化クラス
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

  void setScanPair(const Scan2D *cur, const Scan2D *ref) {
    curScan = cur;
    refScan = ref;
  }

/*
  void setScanPair(const Scan2D *cur, const Scan2D *ref) {
    curScan = cur;
    dass->setRefBase(ref->lps);           // データ対応づけのために参照スキャン点を登録
  }

  void setScanPair(const Scan2D *c, const std::vector<LPoint2D> &refLps) {
    curScan = c;
    dass->setRefBase(refLps);           // データ対応づけのために参照スキャン点を登録
  }
*/
/*
  void setDataAssociator(DataAssociator *dass_) {
    dass = dass_;
  }

  void setCostFunction(CostFunction *cfunc_) {
    popt.setCostFunction(cfunc_);
  }
*/
////////////////////////////////////////////////////////////////////////////////

  double estimatePose(Pose2D &initPose, Pose2D &estPose, Eigen::Matrix3d &cov);
};

#endif
