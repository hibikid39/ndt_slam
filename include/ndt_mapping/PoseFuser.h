#ifndef POSEFUSER_H_
#define POSEFUSER_H_

#include <ros/ros.h>
#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"

class PoseFuser {
private:
  double delTime;  // 時間間隔

  // オドメトリ共分散の係数
  double coeVel;
  double coeOmega;

public:
  PoseFuser() : delTime(0.5), coeVel(0.1), coeOmega(0.1) {
    ros::param::get("delTime", delTime);
    ros::param::get("coeVel", coeVel);
    ros::param::get("coeOmega", coeOmega);
  }

  ~PoseFuser() {}

  //////

  void fusePose(const Pose2D &predPose, const Pose2D &estPose,
                const Pose2D &odoMotion, const Pose2D &lastPose,
                const Eigen::Matrix3d &lastCov, const Eigen::Matrix3d &Qmat,
                Pose2D &fusedPose, Eigen::Matrix3d &cov);

  void calOdometryCovariance(const Pose2D &odoMotion,
                             const Pose2D &lastPose,
                             const Eigen::Matrix3d &lastCov,
                             Eigen::Matrix3d &cov);
};

#endif
