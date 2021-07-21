#include "ndt_mapping/PoseEstimatorICP.h"

// 初期値initPoseを与えて、ICPによりロボット位置の推定値estPoseを求める
double PoseEstimatorICP::estimatePose(Pose2D &initPose, Pose2D &estPose){
  double min = HUGE_VAL;             // コスト最小値 初期値は大きく HUGE_VAL:巨大な値
  popt.setLimit(0.2);                // limitは外れ値の閾値[m]

  double cost = 0;                       // コスト
  double costOld = min;                  // 1つ前の値。収束判定のために使う。
  Pose2D pose = initPose;
  Pose2D poseMin = initPose;             // poseMin:最適ポーズ

  // コストの変化量が小さくなったら終了 i<100は振動対策
  for (int i=0; std::abs(costOld-cost) > thre && i<100; i++){
    if (i > 0)
      costOld = cost;
    // データ対応づけ 結果はDataAssociatorのcurLps,refLpsに格納
    double ratio = dass->findCorrespondence(curScan, pose);    // ratio:対応付けできた割合
    Pose2D newPose;
    popt.setPoints(dass->curLps, dass->refLps);               // 対応結果をコスト関数に渡す
    cost = popt.optimizePose(pose, newPose);                  // その対応づけにおいてロボット位置の最適化

//    printf("pose : pose.tx=%g, pose.ty=%g, pose.th=%g\n", pose.tx, pose.ty, pose.th);
//    printf("newPose : newPose.tx=%g, newPose.ty=%g, newPose.th=%g\n", newPose.tx, newPose.ty, newPose.th);
    pose = newPose;

    if (cost < min) {                                             // コスト最小結果を保存
      poseMin = newPose;
      min = cost;
    }
//    printf("[PoseEstimatorICP] dass->curLps.size=%lu, dass->refLps.size=%lu, ratio=%g\n", dass->curLps.size(), dass->refLps.size(), ratio);
//    printf("[PoseEstimatorICP] i=%d: cost=%g, costOld=%g\n", i, cost, costOld);
  }

  pnrate = popt.getPnrate();
  usedNum = dass->curLps.size();

  estPose = poseMin;

  ROS_INFO("[PoseEstimatorICP::estimatePose] finalError=%g, pnrate=%g, usedNum=%d", min, pnrate, int(usedNum));
//  printf("estPose:  tx=%f, ty=%f, th=%f\n", pose.tx, pose.ty, pose.th);      // 確認用

  if (min < HUGE_VAL)
    totalError += min;                      // 誤差合計
//  printf("totalError=%f\n", totalError);    // 確認用

  return(min);
}
