#include "ndt_mapping/FrontEnd.h"

// 現在スキャンscanを処理する。
void FrontEnd::process(Scan2D &scan) {
  ROS_INFO("[FrontEnd::process] scan.sid = %d", scan.sid);
  if (scan.sid < startFrame) {
    return;
  } else {
    // スキャンマッチ
    smat.matchScan(scan);

    // 全体地図を生成
    if (cnt%keyframeSkip == 0){               // キーフレームのときだけ
/*
      if(cnt == 0) pcmap->setNthre(1);        // 1回目は点数が少ないため
      else pcmap->setNthre(5);
*/
      pcmap->makeGlobalMap();                 // 点群地図の全体地図を生成
    }

    // ポーズグラフにオドメトリアークを追加
/*
    Pose2D curPose = pcmap->getLastPose();  // スキャンマッチングで推定した現在のロボット位置
    if (cnt == 0) {                 // 最初はノードを置くだけ
      pg->addNode(curPose);
    } else {                          // 次からはノードを追加して,アークをノードを生成して繋げる
      Eigen::Matrix3d &cov = smat.getCov();
      makeOdometryArc(curPose, cov);
    }
*/

/*
    // ループ閉じ込み
    if ((cnt > keyframeSkip) && (cnt % keyframeSkip == 0)) {      // キーフレームのときだけ行う
      if (lpd.detectLoop(&scan, curPose, cnt)) {  // ループ検出を起動
        ROS_INFO("[FrontEnd::process] detected loop");
        Pose2D lastPose = backEnd->adjustPoses();       // ループが見つかったらポーズ調整
        backEnd->remakeMaps();                          // 地図やポーズグラフの修正

        pcmap->setLastPose(lastPose);
        smat.remakePoseArray(backEnd->newPoses);
      }
    }
*/

    cnt++;
  }
}

////////////

/*
// アークとノードの生成
bool FrontEnd::makeOdometryArc(Pose2D &curPose, const Eigen::Matrix3d &fusedCov) {
  if (pg->nodes.size() == 0)                             // 念のためのチェック
    return(false);
  PoseNode *lastNode = pg->nodes.back();                 // 直前ノード
  PoseNode *curNode = pg->addNode(curPose);              // ポーズグラフに現在ノードを追加

  // 直前ノードと現在ノードの間にオドメトリアークを張る
  Pose2D &lastPose = lastNode->pose;
  Pose2D relPose;
  Pose2D::calMotion(curPose, lastPose, relPose);   // 現在位置と直前位置の相対位置（移動量）の計算
  ROS_INFO("[FrontEnd::makeOdometryArc] lastPose:  tx=%g, ty=%g, th=%g",
                             lastPose.tx, lastPose.ty, lastPose.th);

  Eigen::Matrix3d cov;
  // 移動量の共分散に変換
  CovarianceCalculator::rotateCovariance(lastPose, fusedCov, cov, true);
  // アークの生成
  PoseArc *arc = pg->makeArc(lastNode->nid, curNode->nid, relPose, cov);
  // ポーズグラフにアークを追加
  pg->addArc(arc);

  return(true);
}
*/
