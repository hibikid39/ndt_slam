#ifndef COVARIANCE_CALCULATOR_H_
#define COVARIANCE_CALCULATOR_H_

#include <ros/ros.h>
#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"

// ICPによる推定値の共分散,および,オドメトリによる推定値の共分散を計算する
class CovarianceCalculator {
private:
  double dd;                      // 数値微分の刻み
  double da;                      // 数値微分の刻み
  double a1;                      // オドメトリ共分散の係数
  double a2;                      // オドメトリ共分散の係数

  double a1_sim;                  // オドメトリ共分散の係数(簡易版)
  double a2_sim;                  // オドメトリ共分散の係数(簡易版)
  double a3_sim;                  // オドメトリ共分散の係数(簡易版)

public:
  CovarianceCalculator() : dd(0.0), da(0.0), a1_sim(0.0), a2_sim(0.0), a3_sim(0.0) {
    ros::param::get("cov_a1", a1_sim);
    ros::param::get("cov_a2", a2_sim);
    ros::param::get("cov_a3", a3_sim);
    ros::param::get("dd", dd);
    ros::param::get("da", da);
  }

  ~CovarianceCalculator() {
  }
/*
  void setAlpha(double a1_, double a2_) {
    a1 = a1_;
    a2 = a2_;
  }
*/

////////

  double calIcpCovariance(const Pose2D &pose, std::vector<const LPoint2D*> &curLps, std::vector<const LPoint2D*> &refLps, Eigen::Matrix3d &cov);
  double calPDistance(const LPoint2D *clp, const LPoint2D *rlp, double tx, double ty, double th);

  void calMotionCovarianceSimple(const Pose2D &motion, double dT, Eigen::Matrix3d &cov);
//  void calMotionCovariance(double th, double dx, double dy, double dth, double dt, Eigen::Matrix3d &cov, bool accum=false);
//  void calUk(double vt, double wt, Eigen::Matrix2d &Uk);
//  void calJxk(double th, double vt, double dt, Eigen::Matrix3d &Jxk);
//  void calJuk(double th, double dt, Eigen::Matrix<double, 3, 2> &Juk);

  double calEigen(const Eigen::Matrix3d &cov, double *vals, double *vec1, double *vec2);

//  static void accumulateCovariance(const Pose2D &curPose, const Pose2D &prevPose, const Eigen::Matrix3d &prevCov, const Eigen::Matrix3d &mcov, Eigen::Matrix3d &curCov);
  static void rotateCovariance(const Pose2D &pose, const Eigen::Matrix3d &cov, Eigen::Matrix3d &icov, bool reverse=false);
};

#endif
