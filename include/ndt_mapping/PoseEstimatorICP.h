#ifndef POSEESTIMATOR_ICP_H_
#define POSEESTIMATOR_ICP_H_

#include <ros/ros.h>
#include <vector>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PoseOptimizer.h"
#include "DataAssociator.h"
#include "Timer.h"

//////

class PoseEstimatorICP{
private:
  const Scan2D *curScan;       // 現在スキャン ポインタ
  size_t usedNum;              // ICPに使われた点数。LoopDetectorで信頼性チェックに使う
  double pnrate;               // 正しく対応づけされた点の比率
  double thre;                 // スコア変化の閾値

  PoseOptimizer popt;         // 最適化クラス

  Timer timer;

public:
  double totalError;           // 誤差合計
  DataAssociator *dass;        // データ対応づけクラス

public:

  PoseEstimatorICP() : usedNum(0), pnrate(0.0), thre(0.0), totalError(0.0){
    ros::param::get("delta_thre", thre);
  }

  ~PoseEstimatorICP() {
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
    dass->setRefBase(ref->lps);           // データ対応づけのために参照スキャン点を登録
  }

  void setScanPair(const Scan2D *c, const std::vector<LPoint2D> &refLps) {
    curScan = c;
    dass->setRefBase(refLps);           // データ対応づけのために参照スキャン点を登録
  }

  void setDataAssociator(DataAssociator *dass_) {
    dass = dass_;
  }

  void setCostFunction(CostFunction *cfunc_) {
    popt.setCostFunction(cfunc_);
  }

////////////////////////////////////////////////////////////////////////////////

  double estimatePose(Pose2D &initPose, Pose2D &estPose);
};

#endif
