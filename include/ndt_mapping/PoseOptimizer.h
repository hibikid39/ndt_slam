#ifndef POSE_OPTIMIZER_H_
#define POSE_OPTIMIZER_H_

#include <ros/ros.h>

#include <boost/math/tools/minima.hpp>
#include <vector>
#include <cstdlib>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "CostFunction.h"

class PoseOptimizer {
public:
  int allN;             // 繰り返し総数。テスト用
  double sum;           // 残差合計。テスト用

private:
  double thre;                // コスト変化閾値。変化量がこれ以下なら繰り返し終了
  double dd;                    // 数値微分の刻み（並進）
  double da;                    // 数値微分の刻み（回転）

  CostFunction *cfunc;          // コスト関数

public:
  PoseOptimizer(): allN(0), sum(0), thre(0.0), dd(0.0), da(0.0){
    ros::param::get("delta_thre", thre);
    ros::param::get("dd", dd);
    ros::param::get("da", da);
  }

  ~PoseOptimizer() {
  }

/////

  void setCostFunction(CostFunction *cfunc_) {
    cfunc = cfunc_;
  }

  void setLimit(double l) {
    cfunc->setLimit(l);
  }

  void setPoints(std::vector<const LPoint2D*> &curLps, std::vector<const LPoint2D*> &refLps) {
    cfunc->setPoints(curLps, refLps);
  }

  double getPnrate() {
    return (cfunc->getPnrate());
  }
/*
  void setDdDa(double d, double a){
    dd = d;
    da = a;
  }
*/

////////

  double optimizePose(Pose2D &initPose, Pose2D &estPose);
  double search(Pose2D &pose, Pose2D &dp);
  double objFunc(double tt, Pose2D &pose, Pose2D &dp);

};

#endif
