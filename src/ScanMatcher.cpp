#include "ndt_mapping/ScanMatcher.h"

// スキャンマッチングの実行
bool ScanMatcher::matchScan(Scan2D &curScan) {
  // スキャン点間隔を均一化する
  spres.resamplePoints(&curScan);

  // spanaが設定されていれば、スキャン点の法線を計算する
  //spana.analysePoints(curScan.lps);

  // 最初のスキャンは単に地図に入れるだけ
  if (cnt == 0) {
    initPose = curScan.pose;
    growMap(curScan, initPose);          // 初期位置で地図生成
    savePose(curScan.header, initPose);  // 初期位置を保存
    tfb.publish_tf_map2odom(initPose);       // 初期位置はodom座標系とmap座標系のずれ
    prevScan = curScan;                      // 直前スキャンの設定
    cnt++;
    return(true);
  }

  // オドメトリから初期値を設定
  // Scanに入っているオドメトリ値を用いて移動量を計算する
  // 移動量 + 1つ前の自己位置 = 現在の自己位置(NDTの初期値)
  Pose2D odoMotion;                                                 // オドメトリに基づく移動量
  Pose2D::calMotion(curScan.pose, prevScan.pose, odoMotion);        // 前スキャンとの相対位置が移動量

  Pose2D lastPose = pcmap->getLastPose();                        // 直前の推定自己位置
  Pose2D predPose;                                               // オドメトリによる予測位置
  Pose2D::calPredPose(odoMotion, lastPose, predPose);               // 直前位置に移動量を加えて予測位置を得る

//  printf("[ScanMatcher2D] lastPose: tx=%f, ty=%f, th=%f\n", lastPose.tx, lastPose.ty, lastPose.th);    // 確認用
//  printf("[ScanMatcher2D] predPose: tx=%f, ty=%f, th=%f\n", predPose.tx, predPose.ty, predPose.th);    // 確認用

  // 参照スキャン生成
  const Scan2D *refScan = refScanMaker.makeRefScan();            // 参照スキャンの生成
  // 現在スキャンと参照スキャンを設定
  estim->setScanPair(&curScan, refScan);
//  printf("[ScanMatcher2D] curScan.size=%lu, refScan.size=%lu\n", curScan.lps.size(), refScan->lps.size());

  // ICP (データ対応付け + ロボット位置最適化)
  Pose2D estPose;                                                // ICPによる推定位置
  double cost = estim->estimatePose(predPose, estPose);          // 予測位置を初期値にしてICPを実行

//  (estim->dass)->clearNntab();

//  size_t usedNum = estim->getUsedNum();

  // 閾値より小さければ成功とする scthre:スコアの閾値 ,nthre:使用点の閾値
  bool successful;                                               // スキャンマッチングに成功したかどうか
  if (cost <= scthre)
    successful = true;
  else
    successful = false;

  ROS_INFO("[ScanMatcher] cost=%g, successful=%d", cost, successful);

/*
  // 退化処理
  if (degCheck) {                         // 退化の対処をする場合
    if (successful) {
      Pose2D fusedPose;                       // 融合結果
      Eigen::Matrix3d fusedCov;               // センサ融合後の共分散
      pfu->setRefScan(refScan);
      // センサ融合器pfuで、ICP結果とオドメトリ値を融合する
      double ratio = pfu->fusePose(&curScan, estPose, odoMotion, lastPose, fusedPose, fusedCov);
      estPose = fusedPose;
      cov = fusedCov; // ポーズグラフに使用
      ROS_INFO("[ScanMatcher2D] pose fused");
      (pfu->dass)->clearNntab();
//      printf("ratio=%g. Pose fused.\n", ratio);     // ratioは退化度 確認用

      // 共分散を累積する
      Eigen::Matrix3d covL;               // 移動量の共分散
      CovarianceCalculator::rotateCovariance(lastPose, fusedCov, covL, true);   // 移動量の共分散に変換
      Eigen::Matrix3d tcov;                // 累積後の共分散
      CovarianceCalculator::accumulateCovariance(lastPose, estPose, totalCov, covL, tcov);
      totalCov = tcov;

    }
    else {
      pfu->calOdometryCovariance(odoMotion, lastPose, cov);       // covはオドメトリ共分散だけ
    }
  }
*/

  // ICP成功でなければ,オドメトリによる予測位置を使う
  if (!successful){
    estPose = predPose;
  }

//  timer.start_timer();

  // 地図生成
  growMap(curScan, estPose);               // 地図にスキャン点群を追加
  prevScan = curScan;                      // 直前スキャンの設定

//  timer.end_timer();
//  timer.print_timer();

  // 自己位置を保存
  savePose(curScan.header, estPose);
  // オドメトリのずれをtfでpublish
  Pose2D errorPose;
  Pose2D::calGlobalMotion(estPose, curScan.pose, errorPose);
  tfb.publish_tf_map2odom(errorPose);

/*
  // 確認用
//  printf("lastPose: tx=%g, ty=%g, th=%g\n", lastPose.tx, lastPose.ty, lastPose.th);
  printf("predPose: tx=%g, ty=%g, th=%g\n", predPose.tx, predPose.ty, predPose.th);     // 確認用
  printf("estPose: tx=%g, ty=%g, th=%g\n", estPose.tx, estPose.ty, estPose.th);
  printf("cov: %g, %g, %g, %g\n", totalCov(0,0), totalCov(0,1), totalCov(1,0), totalCov(1,1));
  printf("mcov: %g, %g, %g, %g\n", pfu->mcov(0,0), pfu->mcov(0,1), pfu->mcov(1,0), pfu->mcov(1,1));
  printf("ecov: %g, %g, %g, %g\n", pfu->ecov(0,0), pfu->ecov(0,1), pfu->ecov(1,0), pfu->ecov(1,1));

  // 共分散の保存（確認用）
//  PoseCov pcov(estPose, cov);
//  PoseCov pcov(estPose, totalCov);
//  PoseCov pcov(estPose, pfu->mcov);
  PoseCov pcov(estPose, pfu->ecov);
  poseCovs.emplace_back(pcov);

  // 累積走行距離の計算（確認用）
  Pose2D estMotion;                                                    // 推定移動量
  Pose2D::calRelativePose(estPose, lastPose, estMotion);
  atd += sqrt(estMotion.tx*estMotion.tx + estMotion.ty*estMotion.ty);
  printf("atd=%g\n", atd);
*/

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

//  printf("[ScanMatcher2D] estPose: tx=%f, ty=%f, th=%f\n", pose.tx, pose.ty, pose.th);    // 確認用

}
