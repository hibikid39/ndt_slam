#include "ndt_slam/ScanMatcher.h"

// スキャンマッチングの実行
bool ScanMatcher::matchScan(Scan2D &curScan) {
  // スキャン点間隔を均一化する
  spres.resamplePoints(&curScan);

  // 最初のスキャンは単に地図に入れるだけ
  if (cnt == 0) {
    ROS_INFO("[ScanMatcher::matchScan] cnt = 0");
    initPose = curScan.pose;
    growMap(curScan, initPose);       // 初期位置で地図生成
    Eigen::Matrix3d cov;              // 姿勢共分散の初期値
    cov << 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0,
           0.0, 0.0, 0.0;
    savePose(curScan.header, initPose, cov);  // 初期位置を保存
    tfb.publish_tf_map2odom(initPose);        // 初期位置はodom座標系とmap座標系のずれ
    prevScan = curScan;                       // 直前スキャンの設定
    cnt++;
    return(true);
  }

  // オドメトリから初期値を設定
  // Scanに入っているオドメトリ値を用いて移動量を計算する
  // 移動量 + 1つ前の自己位置 = 現在の自己位置(NDTの初期値)
  Pose2D odoMotion;                                                 // オドメトリに基づく移動量(ロボット座標系)
  Pose2D::calMotion(curScan.pose, prevScan.pose, odoMotion);        // 前スキャンとの相対位置が移動量

  Pose2D lastPose = pcmap->getLastPose();                        // 直前の推定自己位置
  Pose2D predPose;                                               // オドメトリによる予測位置
  Pose2D::calPredPose(odoMotion, lastPose, predPose);               // 直前位置に移動量を加えて予測位置を得る
  ROS_INFO("[ScanMatcher::matchScan] lastPose.tx=%f, lastPose.ty=%f, lastPose.th=%f", lastPose.tx, lastPose.ty, lastPose.th);
  ROS_INFO("[ScanMatcher::matchScan] predPose.tx=%f, predPose.ty=%f, predPose.th=%f", predPose.tx, predPose.ty, predPose.th);

  // 参照スキャン生成
//  const Scan2D *refScan = refScanMaker.makeRefScan();            // 参照スキャンの生成
  // 現在スキャンと参照スキャンを設定
//  estim->setScanPair(&curScan, refScan);
  estim->setScanPair(&curScan, pcmap->localMap_cloud);

  // NDT
  Pose2D estPose;                                                // 推定位置
  Eigen::Matrix3d Qmat;  // NDTの共分散
  double cost = estim->estimatePose(predPose, estPose, Qmat);    // 予測位置を初期値にしてNDTを実行
  ROS_INFO("[ScanMatcher::matchScan] estPose.tx=%f, estPose.ty=%f, estPose.th=%f", estPose.tx, estPose.ty, estPose.th);

  // 閾値より小さければ成功とする scthre:コストの閾値
  bool successful;              // スキャンマッチングに成功したかどうか
  if (cost <= scthre)
    successful = true;
  else
    successful = false;

  ROS_INFO("[ScanMatcher::matchScan] cost=%g, successful=%d", cost, successful);

  // センサ融合
  Eigen::Matrix3d cov; // 姿勢の共分散
  Pose2D fusedPose;                       // 融合結果
  if (successful) {
    pfu.fusePose(predPose, estPose, odoMotion, lastPose, lastCov, Qmat, fusedPose, cov);
    ROS_INFO("[ScanMatcher::matchScan] pose fused");
  } else {
    pfu.calOdometryCovariance(odoMotion, lastPose, lastCov, cov);
    fusedPose = predPose;
  }
  lastCov = cov;
  ROS_INFO("[ScanMatcher::matchScan] fusedPose.tx=%f, fusedPose.ty=%f, fusedPose.th=%f", fusedPose.tx, fusedPose.ty, fusedPose.th);

//  timer.start_timer();

  // 地図生成
  growMap(curScan, fusedPose);               // 地図にスキャン点群を追加
  prevScan = curScan;                      // 直前スキャンの設定

//  timer.end_timer();
//  timer.print_timer();

  // 自己位置を保存
  savePose(curScan.header, fusedPose, cov);
  // オドメトリのずれをtfでpublish
  Pose2D errorPose;
  Pose2D::calGlobalMotion(fusedPose, curScan.pose, errorPose);
  tfb.publish_tf_map2odom(errorPose);

  ++cnt;
  return(successful);
}

////////////////////

// 現在スキャンを追加して、地図を成長させる
void ScanMatcher::growMap(const Scan2D &scan, const Pose2D &pose) {
  std::vector<LPoint2D> globalLps;  // 地図座標系での点群

  for(int i=0; i<scan.lps.size(); i++) {
    const LPoint2D &lp = scan.lps[i];
    if (lp.type == ISOLATE)                              // 孤立点（法線なし）は除外
      continue;
    double x = pose.Rmat[0][0]*lp.x + pose.Rmat[0][1]*lp.y + pose.tx;         // 地図座標系に変換
    double y = pose.Rmat[1][0]*lp.x + pose.Rmat[1][1]*lp.y + pose.ty;
    double nx = pose.Rmat[0][0]*lp.nx + pose.Rmat[0][1]*lp.ny;           // 法線ベクトルも変換
    double ny = pose.Rmat[1][0]*lp.nx + pose.Rmat[1][1]*lp.ny;

    LPoint2D mlp(scan.sid, x, y);                             // 新規に点を生成
    mlp.setNormal(nx, ny);
    mlp.setType(lp.type);
    globalLps.push_back(mlp);                             // mlpはvector内にコピーされる
  }

  // 点群地図pcmapに登録
  pcmap->addPose(pose);
  pcmap->addPoints(globalLps);
  pcmap->setLastPose(pose);
  pcmap->setLastScan(scan);          // 参照スキャン用に保存
  pcmap->makeLocalMap();             // 局所地図を生成
}
