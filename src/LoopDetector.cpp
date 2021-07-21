#include "ndt_mapping/LoopDetector.h"

using namespace std;

////////////

// ループ検出
// 現在位置curPoseに近く、現在スキャンcurScanに形が一致する場所をロボット軌跡から見つけてポーズアークを張る。
bool LoopDetector::detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt) {
  // 最も近い部分地図を探す
  double atd = pcmap->atd;                   // 現在の実際の累積走行距離
  double atdR = 0;                           // 下記の処理で軌跡をなぞる時の累積走行距離
  const vector<Submap> &submaps = pcmap->submaps;      // 部分地図
  const vector<Pose2D> &poses = pcmap->poses;          // ロボット軌跡
  double dmin=HUGE_VAL;                                // 前回訪問点までの距離の最小値
  size_t imin=0, jmin=0;                               // 距離最小の前回訪問点のインデックス
  Pose2D prevP(0, 0, 0);                               // 直前のロボット位置 最初は初期値

  for (size_t i=0; i<submaps.size()-1; i++) {          // 現在の部分地図以外を探す
    const Submap &submap = submaps[i];                 // i番目の部分地図
    for (size_t j=submap.cntS; j<=submap.cntE; j++) {  // 部分地図の各ロボット位置について
      Pose2D p = poses[j];                             // ロボット位置
      atdR += sqrt((p.tx - prevP.tx)*(p.tx - prevP.tx) + (p.ty - prevP.ty)*(p.ty - prevP.ty));
      if (atd-atdR < atdthre) {     // 現在位置までの走行距離が短いとループとみなさず、もうやめる
        i = submaps.size();         // これで外側のループからも抜ける 関数からも43行目のifで抜ける
        break;
      }
      prevP = p;

      double d = (curPose.tx - p.tx)*(curPose.tx - p.tx) + (curPose.ty - p.ty)*(curPose.ty - p.ty);
      if (d < dmin) {                                  // 現在位置とpとの距離がこれまでの最小か
        dmin = d;
        imin = i;                                      // 候補となる部分地図のインデックス
        jmin = j;                                      // 前回訪問点のインデックス
      }
//      printf("i=%lu, j=%lu: atd=%g, atdR=%g, atdthre=%g\n", i, j, atd, atdR, atdthre);             // 確認用
    }
  }

  ROS_INFO("[LoopDetector::detectLoop] dmin=%g, radius=%g, imin=%lu, jmin=%lu",
                                       sqrt(dmin), radius, imin, jmin);  // 確認用

  if (dmin > radius*radius)          // 前回訪問点までの距離が遠い(閾値外)とループ検出しない,
    return(false);

  Submap &refSubmap = pcmap->submaps[imin];            // 最も近い部分地図を参照スキャンにする
  const Pose2D &initPose = poses[jmin];
  ROS_INFO("[LoopDetector::detectLoop] curPose:  tx=%g, ty=%g, th=%g", curPose.tx, curPose.ty, curPose.th);

  // 再訪点の位置を求める
  vector<Pose2D> candidates;                             // スコアのよい候補位置
  Pose2D revisitPose;                                    // 再訪点
  bool flag = estimateRevisitPose(curScan, refSubmap.lps, initPose, candidates);  // 候補位置 candidates
  if(flag) flag = targetRevisitPose(curScan, refSubmap.lps, candidates, revisitPose);          // 再訪点を1つに絞る

  if (flag) {                                          // ループを検出した
    pfu->setRefLps(refSubmap.lps);
    Eigen::Matrix3d icpCov;                                                  // ICPの共分散
    double ratio = pfu->calIcpCovariance(revisitPose, curScan, icpCov);      // ICPの共分散を計算
    icpCov *= 0.1;
    (pfu->dass)->clearNntab();

    LoopInfo info;                                     // ループ検出結果
    info.pose = revisitPose;                           // ループアーク情報に再訪点位置を設定
    info.cov = icpCov;                                 // ループアーク情報に共分散を設定。
    info.curId = cnt;                                  // 現在位置のノードid
    info.refId = static_cast<int>(jmin);               // 前回訪問点のノードid
    makeLoopArc(info);                                 // ループアーク生成
/*
    // 確認用
    Scan2D refScan;
    Pose2D spose = poses[refSubmap.cntS];
    refScan.setSid(info.refId);
    refScan.setLps(refSubmap.mps);
    refScan.setPose(spose);
    LoopMatch lm(*curScan, refScan, info);
    loopMatches.emplace_back(lm);
    printf("curId=%d, refId=%d\n", info.curId, info.refId);
*/
  }

  return(flag);
}

//////////

// 前回訪問点(refId)を始点ノード、現在位置(curId)を終点ノードにして、ループアークを生成する。
void LoopDetector::makeLoopArc(LoopInfo &info) {
  if (info.arcked)                                             // infoのアークはすでに張ってある
    return;
  info.setArcked(true);

  Pose2D srcPose = pcmap->poses[info.refId];                   // 前回訪問点の位置
  Pose2D dstPose(info.pose.tx, info.pose.ty, info.pose.th);    // 再訪点の位置
  Pose2D relPose;
  Pose2D::calMotion(dstPose, srcPose, relPose);          // ループアークの拘束

  // アークの拘束は始点ノードからの相対位置なので、共分散をループアークの始点ノード座標系に変換
  Eigen::Matrix3d cov;
  CovarianceCalculator::rotateCovariance(srcPose, info.cov, cov, true);    // 共分散の逆回転

  PoseArc *arc = pg->makeArc(info.refId, info.curId, relPose, cov);        // ループアーク生成
  pg->addArc(arc);                                                         // ループアーク登録

/*
  // 確認用
  printf("makeLoopArc: pose arc added\n");
  printf("srcPose: tx=%g, ty=%g, th=%g\n", srcPose.tx, srcPose.ty, srcPose.th);
  printf("dstPose: tx=%g, ty=%g, th=%g\n", dstPose.tx, dstPose.ty, dstPose.th);
  printf("relPose: tx=%g, ty=%g, th=%g\n", relPose.tx, relPose.ty, relPose.th);
  PoseNode *src = pg->findNode(info.refId);
  PoseNode *dst = pg->findNode(info.curId);
  Pose2D relPose2;
  Pose2D::calRelativePose(dst->pose, src->pose, relPose2);
  printf("relPose2: tx=%g, ty=%g, th=%g\n", relPose2.tx, relPose2.ty, relPose2.th);
*/
}

//////////

// 現在スキャンcurScanと部分地図の点群refLpsでICPを行い、再訪点の位置を求める。
bool LoopDetector::estimateRevisitPose(const Scan2D *curScan, const vector<LPoint2D> &refLps, const Pose2D &initPose, vector<Pose2D> &candidates) {
  dass->setRefBase(refLps);                            // データ対応づけ器に参照点群を設定
  cfunc->setLimit(0.2);                                // コスト関数の誤差閾値

  ROS_INFO("[LoopDetector::estimateRevisitPose] initPose: tx=%g, ty=%g, th=%g", initPose.tx, initPose.ty, initPose.th);       // 確認用

//  size_t usedNumMin = 50;
  size_t usedNumMin = 100;

  // 初期位置initPoseの周囲をしらみつぶしに調べる。
  // 効率化のため、ICPは行わず、各位置で単純にマッチングスコアを調べる。
  double rangeT = 2;                              // 並進の探索範囲[m] -2m~2m
  double rangeA = 90;                                    // 回転の探索範囲[度] -90度~90度
  double dd = 0.1;                                       // 並進の探索間隔[m]
  double da = 1;                                         // 回転の探索間隔[度]
  double pnrateMax=0;
  std::vector<double> pnrates;
  double scoreMin=1000000;
  std::vector<double> scores;

  for (double dy=-rangeT; dy<=rangeT; dy+=dd) {          // 並進yの探索繰り返し
    double y = initPose.ty + dy;                         // 初期位置に変位分dyを加える
    for (double dx=-rangeT; dx<=rangeT; dx+=dd) {        // 並進xの探索繰り返し
      double x = initPose.tx + dx;                       // 初期位置に変位分dxを加える
      for (double dth=-rangeA; dth<=rangeA; dth+=da) {   // 回転の探索繰り返し
        double th = MyUtil::add_angle(initPose.th, dth);       // 初期位置に変位分dthを加える
        Pose2D pose(x, y, th);
        double mratio = dass->findCorrespondence(curScan, pose);   // 位置poseでデータ対応づけ
        size_t usedNum = dass->curLps.size();
        //ROS_INFO("usedNum=%lu, mratio=%g", usedNum, mratio);          // 確認用
        if (usedNum < usedNumMin || mratio < 0.7)        // 対応率が悪いと飛ばす
          continue;
        cfunc->setPoints(dass->curLps, dass->refLps);    // コスト関数に点群を設定
        double score =  cfunc->calValue(x, y, th);       // コスト値（マッチングスコア）
        double pnrate = cfunc->getPnrate();              // 詳細な点の対応率
        ROS_INFO("[LoopDetector::estimateRevisitPose] score=%g, pnrate=%g", score, pnrate); // 確認用
        if (pnrate > 0.8) {
          candidates.emplace_back(pose);
          if (score < scoreMin) scoreMin = score;
          scores.push_back(score);
//          printf("pose: tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty, pose.th);  // 確認用
//          printf("score=%g, pnrate=%g\n", score, pnrate);                    // 確認用
        }
      }
    }
  }
  ROS_INFO("[LoopDetector::estimateRevisitPose] candidates.size=%lu", candidates.size());  // 確認用

  dass->clearNntab();

  if (candidates.size() == 0)
    return(false);

  return(true);
}

// 候補位置candidatesの中から最もよいものをICPで選ぶ
bool LoopDetector::targetRevisitPose(const Scan2D *curScan, const vector<LPoint2D> refLps, const vector<Pose2D> candidates, Pose2D &revisitPose) {
  Pose2D best;                                              // 最良候補
  double smin=1000000;                                      // ICPスコア最小値
  size_t usedNumMin = 100;
  estim->setScanPair(curScan, refLps);                      // ICPにスキャン設定

  for (size_t i=0; i<candidates.size(); i++) {
    Pose2D p = candidates[i];                               // 候補位置
    Pose2D estP;
    double score = estim->estimatePose(p, estP);            // ICPでマッチング位置を求める
    double pnrate = estim->getPnrate();                     // ICPでの点の対応率
    size_t usedNum = dass->curLps.size();                  // ICPで使用した点数
    if (score < smin && pnrate >= 0.9 && usedNum >= usedNumMin) {  // ループ検出は条件厳しく
      smin = score;
      best = estP;
      ROS_INFO("smin=%g, pnrate=%g, usedNum=%d", smin, pnrate, int(usedNum));    // 確認用
    }
  }

  (estim->dass)->clearNntab();

  // 最小スコアが閾値より小さければ見つけた
  if (smin <= scthre) {
    revisitPose = best;
    return(true);
  }

  return(false);
}
