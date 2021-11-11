#ifndef POINT_CLOUD_MAP_H_
#define POINT_CLOUD_MAP_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "PCFilter.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "Timer.h"

// 部分地図
struct Submap {
  double atdS;               // 部分地図の始点での累積走行距離
  size_t cntS;               // 部分地図の最初のスキャン番号
  size_t cntE;               // 部分地図の最後のスキャン番号

  //std::vector<LPoint2D> lps; // 部分地図内のスキャン点群
  //Pose2D repPose; // 自己位置の代表点
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud;

  double LeafSize; // フィルタサイズ

  // デバッグ用
  Timer timer;

  Submap() : atdS(0), cntS(0), cntE(-1), LeafSize(0.2) {
    ros::param::get("LeafSize", LeafSize);

    p_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  Submap(double a, size_t s) : cntE(-1), LeafSize(0.2) {
    atdS = a;
    cntS = s;
    ros::param::get("LeafSize", LeafSize);

    p_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  void addPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
    *p_cloud += *cloud_ptr;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterPoints();
};

// 点群地図クラス
class PointCloudMap{
public:
  PCFilter pcf;
//  static const int MAX_POINT_NUM=10000000;          // globalMapの最大点数

  std::vector<Pose2D> poses;            // ロボット軌跡
  Pose2D lastPose;                      // 最後に推定したロボット位置
  Scan2D lastScan;                      // 最後に処理したスキャン
  int startFrame;                       // 開始したフレーム番号

  bool removeDyna;

  pcl::PointCloud<pcl::PointXYZ>::Ptr globalMap_cloud;  // 全体地図 間引き後の点
  pcl::PointCloud<pcl::PointXYZ>::Ptr localMap_cloud;   // 現在位置近傍の局所地図
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scans; // スキャンデータ履歴

  // 部分地図
  double sepThre;                // 部分地図の区切りとなる累積走行距離(atd)[m]
  double atd;                           // 現在の累積走行距離
  std::vector<Submap> submaps;          // 部分地図

  // デバッグ用
  Timer timer;

  // ファイル出力用
  pcl::PointCloud<pcl::PointXYZ> p_cloud;

  PointCloudMap() : sepThre(30), startFrame(0), removeDyna(false) {
    ros::param::get("start_frame", startFrame);
    ros::param::get("sepThre", sepThre);
    ros::param::get("removeDyna", removeDyna);

    globalMap_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    localMap_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    // 最初の部分地図を作っておく
    Submap submap;
    submaps.emplace_back(submap);
  }

  ~PointCloudMap() {
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

  void saveGlobalMap() {
    // 作成したPointCloudをPCD形式で保存する
	  ROS_INFO("[PointCloudMap::saveGlobalMap] savePCDFileASCII");
  	pcl::io::savePCDFileASCII("p_cloud_ascii.pcd", *globalMap_cloud); // テキスト形式で保存する
    ROS_INFO("[PointCloudMap::saveGlobalMap] finish.");
  }

/////////////

  void addPose(const Pose2D &p);
  void addPoints(const std::vector<LPoint2D> &lps);
  void makeGlobalMap();
  void makeLocalMap();
//  void remakeMaps(const std::vector<Pose2D> &newPoses);
};

#endif
