#ifndef POINT_CLOUD_MAP_H_
#define POINT_CLOUD_MAP_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
//#include "NNGridTable.h"
#include "Timer.h"

// 部分地図
struct Submap {
  double atdS;               // 部分地図の始点での累積走行距離
  size_t cntS;               // 部分地図の最初のスキャン番号
  size_t cntE;               // 部分地図の最後のスキャン番号

  std::vector<LPoint2D> lps; // 部分地図内のスキャン点群
  Pose2D repPose; // 自己位置の代表点

  double LeafSize; // フィルタサイズ

  // デバッグ用
  Timer timer;

  Submap() : atdS(0), cntS(0), cntE(-1), LeafSize(0.2) {
    ros::param::get("LeafSize", LeafSize);
  }

  Submap(double a, size_t s) : cntE(-1), LeafSize(0.2) {
    atdS = a;
    cntS = s;
    ros::param::get("LeafSize", LeafSize);
  }

  void addPoints(const std::vector<LPoint2D> &lps_) {
    for (size_t i=0; i<lps_.size(); i++)
      lps.emplace_back(lps_[i]);
  }

//  std::vector<LPoint2D> subsamplePoints(int nthre, NNGridTable *nntab);
  std::vector<LPoint2D> filterPoints();
};

// 点群地図クラス
class PointCloudMap{
public:
  static const int MAX_POINT_NUM=1000000;          // globalMapの最大点数
  int nthre;                                       // 格子テーブルセル点数閾値

  std::vector<Pose2D> poses;            // ロボット軌跡
  Pose2D lastPose;                      // 最後に推定したロボット位置
  Scan2D lastScan;                      // 最後に処理したスキャン
  int startFrame;                       // 開始したフレーム番号

  std::vector<LPoint2D> globalMap;      // 全体地図 間引き後の点
  std::vector<LPoint2D> localMap;       // 現在位置近傍の局所地図 スキャンマッチングに使う

//  NNGridTable *nntab;                    // 格子テーブル

  // 部分地図
  double sepThre;                // 部分地図の区切りとなる累積走行距離(atd)[m]
  double atd;                           // 現在の累積走行距離
  std::vector<Submap> submaps;          // 部分地図

  // デバッグ用
  Timer timer;
  std::vector<std::vector<uint32_t>> colorList; // 点群地図の色

  // ROS用
  sensor_msgs::PointCloud pcmap_ros;         // ROSメッセージ用点群地図

  PointCloudMap() : nthre(1), sepThre(30), startFrame(0) {
    ros::param::get("start_frame", startFrame);
    ros::param::get("sepThre", sepThre);

    globalMap.reserve(MAX_POINT_NUM);       // 最初に確保

    // 最初の部分地図を作っておく
    Submap submap;
    submaps.emplace_back(submap);

    // rviz用 点群地図の色を設定
    pcmap_ros.points.reserve(MAX_POINT_NUM);
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "rgb";
    pcmap_ros.channels.emplace_back(channel);
    pcmap_ros.channels[0].values.reserve(MAX_POINT_NUM);
    std::vector<uint32_t> color{255, 0, 0};
    colorList.push_back(color);
    color[0] = 0; color[1] = 255; color[2] = 0;
    colorList.push_back(color);
    color[0] = 0; color[1] = 0; color[2] = 255;
    colorList.push_back(color);
    color[0] = 255; color[1] = 255; color[2] = 0;
    colorList.push_back(color);
    color[0] = 0; color[1] = 255; color[2] = 255;
    colorList.push_back(color);
    color[0] = 255; color[1] = 0; color[2] = 255;
    colorList.push_back(color);
  }

  ~PointCloudMap() {
  }

////////////////////////////////////////////////////////////////////////////////
/*
  void setNNGridTable(NNGridTable *nntab_) {
    nntab = nntab_;
  }
*/
////////////////////////////////////////////////////////////////////////////////

  void setNthre(int _nthre){
    nthre = _nthre;
  }

  void setLastPose(const Pose2D &p) {
    lastPose = p;
  }

  Pose2D getLastPose() const {
    return(lastPose);
  }

  void setLastScan(const Scan2D &s) {
    lastScan = s;
  }

  std::vector<Submap> &getSubmaps() {
    return(submaps);
  }

/////////////

  void addPose(const Pose2D &p);
  void addPoints(const std::vector<LPoint2D> &lps);
  void makeGlobalMap();
  void makeLocalMap();
//  void samplePoints();
  void remakeMaps(const std::vector<Pose2D> &newPoses);
};

#endif
