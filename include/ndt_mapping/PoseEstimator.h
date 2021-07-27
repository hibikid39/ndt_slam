#ifndef POSEESTIMATOR_ICP_H_
#define POSEESTIMATOR_ICP_H_

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

//////

class PoseEstimator{
private:
  const Scan2D *curScan;       // 現在スキャン ポインタ
  const Scan2D *refScan;
  size_t usedNum;              // ICPに使われた点数。LoopDetectorで信頼性チェックに使う
  double pnrate;               // 正しく対応づけされた点の比率
  double thre;                 // スコア変化の閾値

  double TransformationEpsilon;
  double StepSize;
  double Resolution;
  int MaximumIterations;

  double LeafSize; // フィルタサイズ

//  PoseOptimizer popt;         // 最適化クラス
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  Timer timer;

public:
  double totalError;           // 誤差合計
//  DataAssociator *dass;        // データ対応づけクラス

public:

  PoseEstimator() : usedNum(0), pnrate(0.0), thre(0.0), totalError(0.0)
  ,TransformationEpsilon(0.01), StepSize(0.1), Resolution(1.0), MaximumIterations(35), LeafSize(0.1) {
    ros::param::get("delta_thre", thre);
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

  double getPnrate() {
    return(pnrate);
  }

  size_t getUsedNum() {
    return(usedNum);
  }

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

  double estimatePose(Pose2D &initPose, Pose2D &estPose);
};

#endif
