#ifndef SLAM_LAUNCHER_H_
#define SLAM_LAUNCHER_H_

#include <iostream>  // for debug writing
#include <string>    // useful for reading and writing
#include <fstream>   // ifstream, ofstream
#include <sstream>   // istringstream

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>

#include "MyUtil.h"
#include "FrontEnd.h"
#include "PointCloudMap.h"
#include "PoseGraph.h"
#include "Timer.h"
#include "PoseEstimatorICP.h"
#include "DataAssociator.h"
#include "CostFunction.h"
#include "PoseFuser.h"
#include "NNGridTable.h"

/////////

class SlamLauncher{
public:
  //Pose2D ipose;                    // オドメトリ地図構築の補助データ。初期位置の角度を0にする
  //Pose2D lidarOffset;              // レーザスキャナとロボットの相対位置
  PointCloudMap pcmap;              // 点群地図
  PoseGraph pg;                     // ポーズグラフ
  FrontEnd frontEnd;                // SLAMフロントエンド
  BackEnd backEnd;                  // SLAMバックエンド (ポーズ調整,地図の一括更新)

  PoseEstimatorICP estim;           // ロボット位置推定器
  DataAssociator dass;              // データの対応付け
  CostFunction cfunc;               // コスト関数
  PoseFuser pfu;
  NNGridTable nntab;                // 格子テーブル

  Timer timer;

  int rate;     // メインループのレート
  int drawSkip; // 描画間隔
  bool drawScanOrMap; // スキャンを描画orマップを描画 true:map
  bool pcOrOcg; //点群地図or占有格子地図を出力 true:点群
  int stamp;    // タイムスタンプ
  Scan2D scan;  // 最新のスキャンを格納

  ros::Subscriber sub_odom, sub_scan;
  ros::Publisher pub_ocg, pub_pc, pub_poseArray, pub_marker;

  std::ifstream inputfile;
  std::ofstream outputfile;
  std::string filename_in, filename_out;

//  void callback_odom(const nav_msgs::Odometry &odom_msg);
//  void callback_scan(const sensor_msgs::LaserScan &scan_msg);

public:
  SlamLauncher() : rate(0), drawSkip(0), stamp(0), drawScanOrMap(true), pcOrOcg(true){ // スキャン描画or地図描画を選択
    ros::NodeHandle nh;
/*
    sub_odom = nh.subscribe("odom", 1, &SlamLauncher::callback_odom, this);
    sub_scan = nh.subscribe("scan", 1, &SlamLauncher::callback_scan, this);
*/
    pub_ocg = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    pub_pc = nh.advertise<sensor_msgs::PointCloud>("pcmap_ros", 1);
    pub_poseArray = nh.advertise<geometry_msgs::PoseArray>("poses", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::param::get("main_rate", rate);
    ros::param::get("draw_skip", drawSkip);
    ros::param::get("scan_or_map", drawScanOrMap);
    ros::param::get("pc_or_ocg", pcOrOcg);

    frontEnd.setBackEnd(&backEnd);

    if(drawScanOrMap == true) printf("----- Rviz : Map -----\n");
    else printf("----- matplotlib : Scan -----\n");

    // 入力ファイルを開く
    filename_in = "/home/takahashi/catkin_ws/scan_odom1.txt";
    inputfile.open(filename_in, std::ios::in);
    if(!inputfile) {
      ROS_INFO_STREAM("[ERROR] cannot open " << filename_in);
      exit(1);
    }

    // 出力ファイルを開く
    filename_out = "/home/takahashi/catkin_ws/poses1.txt";
    outputfile.open(filename_out, std::ios::out);
    if(!inputfile) {
      ROS_INFO_STREAM("[ERROR] cannot open " << filename_out);
      exit(1);
    }

    pg.init();
  }
  ~SlamLauncher() {
  }

//////////
  void output_file_poses(std::vector<Pose2D> poses);
  bool input_file_line();
  void init();
  void loop_wait();
};

#endif
