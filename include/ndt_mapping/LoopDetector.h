#ifndef LOOP_DETECTOR
#define LOOP_DETECTOR

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PoseGraph.h"
#include "LoopDetector.h"
#include "PointCloudMap.h"
#include "DataAssociator.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"
///////

struct LoopMatch;

// ループアーク設定情報
struct LoopInfo {
public:
  PoseGraph *pg;                               // ポーズグラフ
  std::vector<LoopMatch> loopMatches;          // デバッグ用
  bool arcked;                   // すでにポーズアークを張ったか
  int curId;                     // 現在キーフレームid（スキャン）
  int refId;                     // 参照キーフレームid（スキャン，または，LocalGridMap2D）
  Pose2D pose;                   // 現在キーフレームが参照キーフレームにマッチするグローバル姿勢（Gridベースの場合は逆）
  double score;                  // ICPマッチングスコア
  Eigen::Matrix3d cov;           // 共分散

  LoopInfo() : arcked(false), curId(-1), refId(-1), score(-1) {
  }

  ~LoopInfo() {
  }

  void setArcked(bool t) {
    arcked = t;
  }
};

//////////////

// デバッグ用データ
struct LoopMatch {
  Scan2D curScan;
  Scan2D refScan;
  LoopInfo info;

  LoopMatch() {
  }

  LoopMatch(Scan2D &cs, Scan2D &rs, LoopInfo &i) {
    curScan = cs;
    refScan = rs;
    info = i;
  }
};
////////////

class LoopDetector {
private:
  PoseGraph *pg;                               // ポーズグラフ
  PointCloudMap *pcmap;                      // 点群地図
  CostFunction *cfunc;                         // コスト関数(ICPとは別に使う)
  PoseEstimatorICP *estim;                     // ロボット位置推定器(ICP)
  DataAssociator *dass;                        // データ対応づけ器
  PoseFuser *pfu;                              // センサ融合器

  std::vector<LoopMatch> loopMatches;          // デバッグ用

  double radius;                               // 探索半径[m]（現在位置と再訪点の距離閾値）
  double atdthre;                              // 累積走行距離の差の閾値[m]
  double scthre;                               // ICPスコアの閾値

public:
  LoopDetector() : radius(2), atdthre(10), scthre(0.2) {
  }
  ~LoopDetector() {
  }

/////////

  void setPoseFuser(PoseFuser *pfu_) {
    pfu = pfu_;
  }

  void setCostFunction(CostFunction *cfunc_) {
    cfunc = cfunc_;
  }

  void setPoseEstimatorICP(PoseEstimatorICP *estim_) {
    estim = estim_;
  }

  void setDataAssociator(DataAssociator *dass_) {
    dass = dass_;
  }

  void setPointCloudMap(PointCloudMap *pcmap_) {
    pcmap = pcmap_;
  }

  void setPoseGraph(PoseGraph *pg_) {
    pg = pg_;
  }

////////////////////////////////////////////////////////////////////////////////

  // デバッグ用
  std::vector<LoopMatch> &getLoopMatches() {
    return(loopMatches);
  }

////////////////////////////////////////////////////////////////////////////////

  bool detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt);
  void makeLoopArc(LoopInfo &info);
  bool estimateRevisitPose(const Scan2D *curScan, const std::vector<LPoint2D> &refLps, const Pose2D &initPose, std::vector<Pose2D>  &candidates);
  bool targetRevisitPose(const Scan2D *curScan, const std::vector<LPoint2D> refLps, const std::vector<Pose2D> candidates, Pose2D &revisitPose);
};

#endif
