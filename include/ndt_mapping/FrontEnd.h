#ifndef FRONT_END_H_
#define FRONT_END_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>

#include "MyUtil.h"
#include "Scan2D.h"
#include "ScanMatcher.h"
#include "PointCloudMap.h"
#include "Timer.h"
//#include "BackEnd.h"
//#include "LoopDetector.h"
//#include "PoseGraph.h"

// SLAMフロントエンド。ロボット位置推定、地図生成、ループ閉じ込みを取り仕切る。
class FrontEnd {
private:
  int cnt;                           // 論理時刻
  int keyframeSkip;                  // キーフレーム間隔
  int startFrame;                    // 開始するフレーム番号

  ScanMatcher smat;                // スキャンマッチング
  PointCloudMap *pcmap;              // 点群地図
//  PoseGraph *pg;                     // ポーズグラフ
//  LoopDetector lpd;                  // ループ検出器
//  BackEnd *backEnd;

  Timer timer;

public:
  FrontEnd() : cnt(0), keyframeSkip(0), startFrame(0) {
    ros::param::get("keyframe_skip", keyframeSkip);
    ros::param::get("start_frame", startFrame);
  }
  ~FrontEnd() {
  }

///////////////////////////////////////////////////////////////////////////////

  // rvizにpublishするのに使う
  geometry_msgs::PoseArray get_poseArray() {
    return smat.get_poseArray();
  }

  // ファイル書き出しに使う
  std::vector<Pose2D> get_poses() {
    return smat.poses;
  }

///////////////////////////////////////////////////////////////////////////////

  // 地図のファイル書き出し
  void saveMap() {
    pcmap->makeGlobalMap();
    pcmap->saveGlobalMap();
  }

/*
  void setCostFunction(CostFunction *cfunc_) {
    lpd.setCostFunction(cfunc_);
  }

  void setDataAssociator(DataAssociator *dass_) {
    lpd.setDataAssociator(dass_);
  }

  void setPoseEstimatorICP(PoseEstimatorICP *estim_) {
    smat.setPoseEstimatorICP(estim_);
    lpd.setPoseEstimatorICP(estim_);
  }
*/
  void setPoseEstimator(PoseEstimator *estim_) {
    smat.setPoseEstimator(estim_);
  }
  void setPointCloudMap(PointCloudMap *pcmap_) {
    pcmap = pcmap_;
    smat.setPointCloudMap(pcmap_);
//    lpd.setPointCloudMap(pcmap_);
  }
/*
  void setPoseFuser(PoseFuser *pfu_) {
    smat.setPoseFuser(pfu_);
    lpd.setPoseFuser(pfu_);
  }

  void setPoseGraph(PoseGraph *pg_) {
    pg = pg_;
    lpd.setPoseGraph(pg_);
  }

  void setBackEnd(BackEnd *backEnd_){
    backEnd = backEnd_;
  }
*/

///////////////////////////////////////////////////////////////////////////////

  void process(Scan2D &scan);
//  bool makeOdometryArc(Pose2D &curPose, const Eigen::Matrix3d &fusedCov);

};

#endif
