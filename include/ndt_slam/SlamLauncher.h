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

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "MyUtil.h"
#include "FrontEnd.h"
#include "PointCloudMap.h"
//#include "PoseGraph.h"
#include "Timer.h"
#include "PoseEstimator.h"
//#include "DataAssociator.h"
//#include "CostFunction.h"
//#include "PoseFuser.h"
//#include "NNGridTable.h"

/////////

class SlamLauncher{
private:
  int end_frame; // 終了するフレーム番号
public:
  PointCloudMap pcmap;              // 点群地図
  FrontEnd frontEnd;                // SLAMフロントエンド
  PoseEstimator estim;           // ロボット位置推定器

  Timer timer;

  int rate;     // メインループのレート
  int drawSkip; // 描画間隔
  int stamp;    // タイムスタンプ
  Scan2D scan;  // 最新のスキャンを格納

  ros::Subscriber sub_odom, sub_scan;
  ros::Publisher pub_pc, pub_poseArray;

  std::ifstream inputfile;
  std::ofstream outputfile;
  std::string filename_in, poses_name;

  bool sidelidar;

  SlamLauncher() : rate(0), drawSkip(0), stamp(0), end_frame(100), sidelidar(true) { // スキャン描画or地図描画を選択
    ros::NodeHandle nh;
    pub_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("pcmap", 1);
    pub_poseArray = nh.advertise<geometry_msgs::PoseArray>("poses", 1);

    ros::param::get("sidelidar", sidelidar);
    
    ros::param::get("end_frame", end_frame);
    ros::param::get("draw_skip", drawSkip);

    ros::param::get("filename_in", filename_in);
    ros::param::get("poses_name", poses_name);

    // 入力ファイルを開く
    inputfile.open(filename_in, std::ios::in);
    if(!inputfile) {
      ROS_INFO_STREAM("[ERROR] cannot open " << filename_in);
      exit(1);
    }

    // 出力ファイルを開く
    outputfile.open(poses_name, std::ios::out);
    if(!inputfile) {
      ROS_INFO_STREAM("[ERROR] cannot open " << poses_name);
      exit(1);
    }
  }

  ~SlamLauncher() {
  }
  
  void readFormat() {
    std::string buf;
    std::getline(inputfile, buf);
    std::cout << buf << std::endl;
    std::getline(inputfile, buf);
    std::cout << buf << std::endl;
    std::getline(inputfile, buf);
    std::cout << buf << std::endl;
    std::getline(inputfile, buf);
    std::cout << buf << std::endl;
  }

//////////
  void output_file_poses(std::vector<Pose2D> poses);
  bool input_file_line();
  void init();
  void loop_wait();
};

#endif
