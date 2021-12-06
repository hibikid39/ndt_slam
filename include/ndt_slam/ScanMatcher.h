#ifndef SCAN_MATCHER2D_H_
#define SCAN_MATCHER2D_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <vector>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"
#include "ScanPointResampler.h"
//#include "ScanPointAnalyser.h"
#include "PoseEstimator.h"
#include "PoseFuser.h"
#include "TFBroadcaster.h"
#include "Timer.h"

// ICPを用いてスキャンマッチングを行う
class ScanMatcher {
private:
  int cnt;                                // 論理時刻。スキャン番号に対応
  Scan2D prevScan;                        // 1つ前のスキャン
  Pose2D initPose;                        // 地図の原点の位置。通常(0,0,0)

  double scthre;                          // スコア閾値

public:
  PoseEstimator *estim;                 // ロボット位置推定器
  PointCloudMap *pcmap;                   // 点群地図
  ScanPointResampler spres;               // スキャン点間隔均一化
//  RefScanMaker refScanMaker;              // 参照スキャン生成
  TFBroadcaster tfb;                      // 座標変換TFのブロードキャスター

  PoseFuser pfu;                          // センサ融合器
  std::vector<Pose2D> poses;              // 姿勢の履歴
  std::vector<Eigen::Matrix3d> Covs;      // 姿勢の共分散の履歴
  Eigen::Matrix3d lastCov;

  geometry_msgs::PoseArray poseArray;     // RvizのPoseArray用

  Timer timer;

public:
  ScanMatcher() : cnt(0), scthre(0.0){
    ros::param::get("score_thre", scthre);
  }

  ~ScanMatcher(){
  }

///////////

  void setPoseEstimator(PoseEstimator *estim_) {
    estim = estim_;
  }

  void setPointCloudMap(PointCloudMap *pcmap_) {
    pcmap = pcmap_;
//    refScanMaker.setPointCloudMap(pcmap_); // 参照スキャン生成器に地図を渡す
  }

  void savePose(const std_msgs::Header &header, const Pose2D &pose, const Eigen::Matrix3d &cov) {
    poses.push_back(pose);
    Covs.push_back(cov);

    // ROS用 geometry_msgs::PoseArray
    geometry_msgs::Pose po;
    po.position.x = pose.tx;
    po.position.y = pose.ty;
    po.position.z = 0.0;
    // 4元数に変換してorientationに代入
    po.orientation = MyUtil::rpy_to_geometry_quat(0.0, 0.0, DEG2RAD(pose.th));
    poseArray.poses.push_back(po);
    poseArray.header = header;
    poseArray.header.frame_id = "map";
  }

  void remakePoseArray(std::vector<Pose2D> &poses_) {
    poseArray.poses.clear();
    poses.clear();

    for (int i=0; i<poses_.size(); i++) {
      Pose2D pose = poses_[i];
      geometry_msgs::Pose po;
      po.position.x = pose.tx;
      po.position.y = pose.ty;
      po.position.z = 0.0;
      // 4元数に変換してorientationに代入
      po.orientation = MyUtil::rpy_to_geometry_quat(0.0, 0.0, DEG2RAD(pose.th));
      poseArray.poses.push_back(po);

      poses.push_back(pose);
    }
    poseArray.header.stamp = ros::Time::now();
    poseArray.header.frame_id = "map";
  }

  geometry_msgs::PoseArray get_poseArray() {
    return poseArray;
  }

//////////

  bool matchScan(Scan2D &scan);
  void growMap(const Scan2D &scan, const Pose2D &pose);

};

#endif
