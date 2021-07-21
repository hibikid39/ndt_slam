#ifndef POSE_FUSER_H_
#define POSE_FUSER_H_

#include <boost/array.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "DataAssociator.h"
#include "CovarianceCalculator.h"

// センサ融合器。ICPとオドメトリの推定値を融合する。
class PoseFuser {
public:
  Eigen::Matrix3d ecov;                      // ICPの共分散行列
  Eigen::Matrix3d mcov;                      // オドメトリの共分散行列
  Eigen::Matrix3d totalCov;

  DataAssociator *dass;                      // データ対応づけ器
  CovarianceCalculator cvc;                  // 共分散計算器

  ros::Publisher pub_poseWithCov_icp;
  ros::Publisher pub_poseWithCov_m;
  geometry_msgs::PoseWithCovarianceStamped poseWithCov_icp;
  geometry_msgs::PoseWithCovarianceStamped poseWithCov_m;

public:
  PoseFuser() {
    ros::NodeHandle nh;
    pub_poseWithCov_icp = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("poseWithCov_icp", 1);
    pub_poseWithCov_m = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("poseWithCov_m", 1);
    poseWithCov_icp.header.frame_id = "map";
    poseWithCov_m.header.frame_id = "map";
  }

  ~PoseFuser() {
  }

/////

  void setDataAssociator(DataAssociator *dass_) {
    dass = dass_;
  }

  void setRefScan(const Scan2D *refScan) {
    dass->setRefBase(refScan->lps);
  }

  void setRefLps(const std::vector<LPoint2D> &refLps) {
    dass->setRefBase(refLps);
  }

  // ICPの共分散行列の計算 オドメトリの共分散を使用しない場合に使用 setRefLpsの後に行うこと
  double calIcpCovariance(const Pose2D &estMotion, const Scan2D *curScan, Eigen::Matrix3d &cov) {
    dass->findCorrespondence(curScan, estMotion);

    // ICPの共分散 ここで得られるのは 世界座標系での共分散
    double ratio = cvc.calIcpCovariance(estMotion, dass->curLps, dass->refLps, cov);
    return(ratio);
  }

//////////

  double fusePose(Scan2D *curScan, const Pose2D &estPose, const Pose2D &odoMotion, const Pose2D &lastPose, Pose2D &fusedPose, Eigen::Matrix3d &cov);
  void calOdometryCovariance(const Pose2D &odoMotion, const Pose2D &lastPose, Eigen::Matrix3d &mcov);
  double fuse(const Eigen::Vector3d &mu1, const Eigen::Matrix3d &cv1,  const Eigen::Vector3d &mu2, const Eigen::Matrix3d &cv2, Eigen::Vector3d &mu, Eigen::Matrix3d &cv);

};

#endif
