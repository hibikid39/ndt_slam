#include "ndt_mapping/PoseFuser.h"

void PoseFuser::fusePose(const Pose2D &predPose, const Pose2D &estPose,
                         const Pose2D &odoMotion, const Pose2D &lastPose,
                         const Eigen::Matrix3d &lastCov, const Eigen::Matrix3d &Qmat,
                         Pose2D &fusedPose, Eigen::Matrix3d &cov) {

  // 遷移step
  Eigen::Matrix3d cov_hat;  // 遷移後の共分散
  calOdometryCovariance(odoMotion, lastPose, lastCov, cov_hat); // 予測step
  Eigen::Matrix<double,3,1> mu_hat; // 遷移後の姿勢
  mu_hat << predPose.tx, predPose.ty, DEG2RAD(predPose.th);

  std::cout << "cov_hat = " << std::endl;
  std::cout << cov_hat << std::endl;

  // 観測step
  Eigen::Matrix3d Kmat;  // カルマンゲイン
  Kmat = cov_hat*(Qmat+cov_hat).inverse();

  Eigen::Matrix3d Imat;
  Imat << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;
  cov = (Imat - Kmat)*cov_hat;  // 観測step後の共分散

  std::cout << "cov = " << std::endl;
  std::cout << cov << std::endl;

  Eigen::Matrix<double,3,1> zh; // z-h()
  zh << estPose.tx-predPose.tx, estPose.ty-predPose.ty, DEG2RAD(MyUtil::sub_angle(estPose.th,predPose.th));
  Eigen::Matrix<double,3,1> mu; // 観測step後の姿勢
  mu = Kmat*zh + mu_hat;

  fusedPose.setPose(mu(0,0), mu(1,0), RAD2DEG(mu(2,0)));
}

void PoseFuser::calOdometryCovariance(const Pose2D &odoMotion,
                                      const Pose2D &lastPose,
                                      const Eigen::Matrix3d &lastCov,
                                      Eigen::Matrix3d &cov) {

  double v = odoMotion.calDistance()/delTime;
  double omega = DEG2RAD(odoMotion.th/delTime);

  Eigen::Matrix2d Mmat;
  Mmat << coeVel*v*v, 0.0,
         0.0, coeOmega*omega*omega;

  Eigen::Matrix<double,3,2> Amat;
  Amat << delTime*std::cos(DEG2RAD(lastPose.th)), 0.0,
          delTime*std::sin(DEG2RAD(lastPose.th)), 0.0,
          0.0, delTime;

  Eigen::Matrix3d Fmat;
  Fmat << 1.0, 0.0, -v*delTime*std::sin(DEG2RAD(lastPose.th)),
          0.0, 1.0, v*delTime*std::cos(DEG2RAD(lastPose.th)),
          0.0, 0.0, 1.0;

  cov = Fmat*lastCov*Fmat.transpose() + Amat*Mmat*Amat.transpose();
}
