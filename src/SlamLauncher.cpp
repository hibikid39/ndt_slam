#include <iostream>
#include <ros/ros.h>
#include <vector>

#include "ndt_mapping/SlamLauncher.h"

void SlamLauncher::init() {
//  frontEnd.setDataAssociator(&dass);
//  frontEnd.setPoseFuser(&pfu);
//  frontEnd.setCostFunction(&cfunc);
//  frontEnd.setPoseEstimatorICP(&estim);
  frontEnd.setPoseEstimator(&estim);
  frontEnd.setPointCloudMap(&pcmap);
//  frontEnd.setPoseGraph(&pg);
//  frontEnd.setBackEnd(&backEnd);

//  backEnd.setPointCloudMap(&pcmap);
//  backEnd.setPoseGraph(&pg);

//  estim.setCostFunction(&cfunc);
//  estim.setDataAssociator(&dass);

//  pfu.setDataAssociator(&dass);

//  pcmap.setNNGridTable(&nntab);

//  dass.setNNGridTable(&nntab);
}

void SlamLauncher::output_file_poses(std::vector<Pose2D> poses) {
  int stamp = 1;
  outputfile << poses.size() << std::endl;
  for (int i=0; i<poses.size(); i++){
    outputfile << "POSE " << stamp << " " << poses[i].tx << " " << poses[i].ty << " " << poses[i].th << std::endl;
    stamp++;
  }
}

bool SlamLauncher::input_file_line() {
  std::string buf;
  std::string buf_angle;
  std::string buf_range;
  int data_num;
  std::vector<LPoint2D> lps;
  LPoint2D lp;

  if(stamp == 0) std::getline(inputfile, buf, ' '); // LaserScan 最初だけLASERSCANを空読み
  std::getline(inputfile, buf, ' '); // time stamp
  stamp = std::stoi(buf);
  scan.sid = stamp;
  std::getline(inputfile, buf, ' '); // data_num
  data_num = std::stoi(buf);

  lps.reserve(data_num);
  lp.sid = stamp;

  for (int i = 0; i < data_num; i++) {
    std::getline(inputfile, buf_angle, ' ');
    std::getline(inputfile, buf_range, ' ');

    lp.set_RangeAngle2XY(std::stod(buf_range), std::stod(buf_angle));
    lps.push_back(lp);

    if(inputfile.eof()) {
      ROS_INFO("[ERROR] data_num is fault");
      exit(1);
    }
  }
  scan.lps = lps;

  std::getline(inputfile, buf, ' '); // x
  scan.pose.tx = std::stod(buf);
  std::getline(inputfile, buf, ' '); // y
  scan.pose.ty = std::stod(buf);
  std::getline(inputfile, buf, ' '); // theta[deg]
  scan.pose.th = std::stod(buf);
  scan.pose.calRmat();

  if(inputfile.eof()){  // 全て読み込んだら終了
    inputfile.close();
    return true;
  }

  ROS_INFO("[SlamLauncher::input_file_line] scan.pose=(%g, %g, %g)", scan.pose.tx, scan.pose.tx, scan.pose.th);
  ROS_INFO("[SlamLauncher::input_file_line] time_stamp=%d, data_num=%d", stamp, data_num);

  return false;
}

void SlamLauncher::loop_wait() {
  static int cnt = 1;

  while(ros::ok()){
    if(cnt > end_frame) { // 終了時間
      output_file_poses(frontEnd.get_poses());
      frontEnd.saveMap();
      return;
    }

    ROS_INFO("--- [SlamLauncher::loop_wait] cnt=%d ---",cnt);

    if(input_file_line() == true) {
      ROS_INFO("[SlamLauncher::loop_wait] Data Finish.");
      output_file_poses(frontEnd.get_poses());
      frontEnd.saveMap();
      return;
    }

    frontEnd.process(scan); // 処理本体

    if(cnt%drawSkip == 0){                           // drawSkipおきに描画
      pcmap.globalMap_cloud->header.frame_id = "map";
      pcl_conversions::toPCL(ros::Time::now(), pcmap.globalMap_cloud->header.stamp);
      pub_pc.publish(pcmap.globalMap_cloud);
/*
      pcmap.pcmap_ros.header.seq = stamp;
      pcmap.pcmap_ros.header.stamp = ros::Time::now();;
      pcmap.pcmap_ros.header.frame_id = "map";   // フレームIDをworld座標系(map)に変更
      pub_pc.publish(pcmap.pcmap_ros);  // 地図描画
*/
    }
    pub_poseArray.publish(frontEnd.get_poseArray()); // 姿勢描画

    cnt++;
  }

  return;
}
