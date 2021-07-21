#include "ndt_mapping/PoseFuser.h"

using namespace std;

////////////// 逐次SLAM用のセンサ融合 ////////////////

// 逐次SLAMでのICPとオドメトリの推定移動量を融合する dassに参照スキャンを入れておくこと covに移動量の共分散行列が入る。
double PoseFuser::fusePose(
    Scan2D *curScan,
    const Pose2D &estPose,
    const Pose2D &odoMotion,
    const Pose2D &lastPose,
    Pose2D &fusedPose,          // 出力:融合した姿勢
    Eigen::Matrix3d &fusedCov)  // 出力:融合した共分散行列
{
//  printf("[PoseFuser::fusePose] \n");
  // ICPの共分散 ecov
  // 推定位置estPoseでの 現在スキャン点群と参照スキャン点群の対応づけ
  dass->findCorrespondence(curScan, estPose);
  // 地図座標系での位置の共分散 ecov:ICPでの共分散
  double ratio = cvc.calIcpCovariance(estPose, dass->curLps, dass->refLps, ecov);

  // オドメトリの共分散 mcov
  // 速度運動モデルを使うと短期間では共分散が小さすぎるため,簡易版で大きめに計算する
  Eigen::Matrix3d mcovL;
  double dT=0.5;
  cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL);  // オドメトリで得た移動量の共分散Σu（簡易版）
  // 現在位置estPoseで回転させて,地図座標系での共分散mcovを得る (回転方向のみ地図座標系に変換)
  CovarianceCalculator::rotateCovariance(estPose, mcovL, mcov);

  // ecov,mcovともに局所座標系での値
  Eigen::Vector3d mu1(estPose.tx, estPose.ty, DEG2RAD(estPose.th));     // ICPによる推定値
  Pose2D predPose;                                                      // オドメトリによる推定値(Pose2D)
  Pose2D::calPredPose(odoMotion, lastPose, predPose);                 // 直前位置lastPoseに移動量を加えて推定
  Eigen::Vector3d mu2(predPose.tx, predPose.ty, DEG2RAD(predPose.th));  // オドメトリによる推定値

  // 2つの正規分布の融合
  Eigen::Vector3d mu;
  fuse(mu1, ecov, mu2, mcov, mu, fusedCov);

  //MyUtil::printMatrix(mcov);
  
  // 共分散を誤差楕円でpublish
  poseWithCov_icp.pose.pose.position.x = estPose.tx;
  poseWithCov_icp.pose.pose.position.y = estPose.ty;
  poseWithCov_icp.pose.pose.position.z = 0;
  poseWithCov_icp.pose.pose.orientation = MyUtil::rpy_to_geometry_quat(0.0, 0.0, DEG2RAD(estPose.th));
  boost::array<double, 36> cov36_icp;
  MyUtil::convertCov(ecov, cov36_icp);
  poseWithCov_icp.pose.covariance = cov36_icp;
  pub_poseWithCov_icp.publish(poseWithCov_icp);

  poseWithCov_m.pose.pose.position.x = predPose.tx;
  poseWithCov_m.pose.pose.position.y = predPose.ty;
  poseWithCov_m.pose.pose.position.z = 0;
  poseWithCov_m.pose.pose.orientation = MyUtil::rpy_to_geometry_quat(0.0, 0.0, DEG2RAD(predPose.th));
  boost::array<double, 36> cov36_m;
  MyUtil::convertCov(mcov, cov36_m);
  poseWithCov_m.pose.covariance = cov36_m;
  pub_poseWithCov_m.publish(poseWithCov_m);

  //double vals_m[2], vec1_m[2], vec2_m[2];      // 移動量の共分散の固有値 固有ベクトル
  //cvc.calEigen(mcov, vals_m, vec1, vec2);
  //double vals_icp[2], vec1_icp[2], vec2_icp[2];      // ICPの共分散の固有値 固有ベクトル
  //cvc.calEigen(ecov, vals_icp, vec1_icp, vec2_icp);

  fusedPose.setPose(mu[0], mu[1], RAD2DEG(mu[2]));           // 融合した移動量を格納

  totalCov = fusedCov;

  // 確認用
/*
  double vals[2], vec1[2], vec2[2];
  printf("ecov: det=%g, ", ecov.determinant());
  cvc.calEigen(ecov, vals, vec1, vec2);
  printf("mcov: det=%g, ", mcov.determinant());
  cvc.calEigen(mcov, vals, vec1, vec2);
  printf("fusedCov: det=%g, ", fusedCov.determinant());
  cvc.calEigen(fusedCov, vals, vec1, vec2);
*/
//  printf("predPose: tx=%g, ty=%g, th=%g\n", predPose.tx, predPose.ty, predPose.th);
//  printf("estPose: tx=%g, ty=%g, th=%g\n", estPose.tx, estPose.ty, estPose.th);
//  printf("fusedPose: tx=%g, ty=%g, th=%g\n", fusedPose.tx, fusedPose.ty, fusedPose.th);

  return(ratio);
}
// オドメトリの共分散のみ計算 (ICP失敗時に使用)
void PoseFuser::calOdometryCovariance(const Pose2D &odoMotion, const Pose2D &lastPose, Eigen::Matrix3d &mcov) {
  Eigen::Matrix3d mcovL;
  double dT=0.5;
  cvc.calMotionCovarianceSimple(odoMotion, dT, mcovL);                             // オドメトリで得た移動量の共分散（簡易版）
  CovarianceCalculator::rotateCovariance(lastPose, mcovL, mcov);                   // 直前位置lastPoseで回転させて、位置の共分散mcovを得る
}

/////// ガウス分布の融合 ///////

// 2つの正規分布を融合する
double PoseFuser::fuse(
    const Eigen::Vector3d &mu1, // 平均1
    const Eigen::Matrix3d &cv1, // 共分散行列1
    const Eigen::Vector3d &mu2, // 平均2
    const Eigen::Matrix3d &cv2, // 共分散行列2
    Eigen::Vector3d &mu,        // 出力:融合後の平均
    Eigen::Matrix3d &cv)        // 出力:融合した共分散行列
{
  // 共分散行列の融合
  Eigen::Matrix3d IC1 = MyUtil::svdInverse(cv1);
  Eigen::Matrix3d IC2 = MyUtil::svdInverse(cv2);
  Eigen::Matrix3d IC = IC1 + IC2;
  cv = MyUtil::svdInverse(IC);

  // 角度の補正 融合時に連続性を保つため。
  Eigen::Vector3d mu11 = mu1;             // ICPの方向をオドメトリに合せる
  double da = mu2(2) - mu1(2);
  if (da > M_PI)        // mu1が小さい
    mu11(2) += 2*M_PI;
  else if (da < -M_PI)  // mu1が大きい
    mu11(2) -= 2*M_PI;

  // 平均の融合 mu = cv*(cv1^(-1)*mu1 + cv2^(-1)*mu2)
  Eigen::Vector3d nu1 = IC1*mu11;
  Eigen::Vector3d nu2 = IC2*mu2;
  Eigen::Vector3d nu3 = nu1 + nu2;
  mu = cv*nu3;

  // 角度の補正。(-pi, pi)に収める
  if (mu(2) > M_PI)
    mu(2) -= 2*M_PI;
  else if (mu(2) < -M_PI)
    mu(2) += 2*M_PI;

  // 係数部の計算
  Eigen::Vector3d W1 = IC1*mu11;
  Eigen::Vector3d W2 = IC2*mu2;
  Eigen::Vector3d W = IC*mu;
  double A1 = mu1.dot(W1);
  double A2 = mu2.dot(W2);
  double A = mu.dot(W);
  double K = A1+A2-A;

/*
  printf("cv1: det=%g\n", cv1.determinant());
  MyUtil::printMatrix(cv1);
  printf("cv2: det=%g\n", cv2.determinant());
  MyUtil::printMatrix(cv2);
  printf("cv: det=%g\n", cv.determinant());
  MyUtil::printMatrix(cv);
*/

  return(K);
}
