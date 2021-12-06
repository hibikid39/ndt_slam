#include <iostream>
#include <ros/ros.h>
#include <vector>

#include "ndt_slam/SlamLauncher.h"

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
  outputfile << poses.size() << std::endl;
  for (int i=0; i<poses.size(); i+=10){
    outputfile << poses[i].tx << " " << poses[i].ty << " " << poses[i].th << " " << std::endl;
  }
}

bool SlamLauncher::input_file_line() {
  std::string buf;
  std::vector<LPoint2D> lps;
  LPoint2D lp;

  std::getline(inputfile, buf, ' '); // time stamp
  stamp = std::stoi(buf);
  scan.sid = stamp;

  std::getline(inputfile, buf, ' '); // x
  scan.pose.tx = std::stod(buf);
  std::getline(inputfile, buf, ' '); // y
  scan.pose.ty = std::stod(buf);
  std::getline(inputfile, buf, ' '); // theta[deg]
  scan.pose.th = std::stod(buf);

  std::getline(inputfile, buf); // image_name
  std::string image_name = buf;

  std::getline(inputfile, buf, ' '); // data_num(front)
  int data_num_front = std::stoi(buf);
  for (int i = 0; i < data_num_front; i++) {
    std::string buf_x, buf_y;
    std::getline(inputfile, buf_x, ' ');
    std::getline(inputfile, buf_y, ' ');
    double x = std::stod(buf_x);
    double y = std::stod(buf_y);
    lp.setData(stamp, x, y);
    lps.push_back(lp);
  }
  
  std::getline(inputfile, buf, ' '); // data_num(left)
  int data_num_left = std::stoi(buf);
  for (int i = 0; i < data_num_left; i++) {
    std::string buf_x, buf_y;
    std::getline(inputfile, buf_x, ' ');
    std::getline(inputfile, buf_y, ' ');
    double x = std::stod(buf_x);
    double y = std::stod(buf_y);
    lp.setData(stamp, x, y);
    if(sidelidar == true) lps.push_back(lp);
  }

  std::getline(inputfile, buf, ' '); // data_num(right)
  int data_num_right = std::stoi(buf);
  for (int i = 0; i < data_num_right; i++) {
    std::string buf_x, buf_y;
    std::getline(inputfile, buf_x, ' ');
    std::getline(inputfile, buf_y, ' ');
    double x = std::stod(buf_x);
    double y = std::stod(buf_y);
    lp.setData(stamp, x, y);
    if(sidelidar == true) lps.push_back(lp);
  }

  scan.lps = lps;

  scan.pose.calRmat();

  if(inputfile.eof()){  // 全て読み込んだら終了
    inputfile.close();
    return true;
  }

  ROS_INFO("[SlamLauncher::input_file_line] scan.pose=(%g, %g, %g)", scan.pose.tx, scan.pose.tx, scan.pose.th);
  ROS_INFO("[SlamLauncher::input_file_line] time_stamp=%d, data_num=%d", stamp, lps.size());

  return false;
}

void SlamLauncher::loop_wait() {
  static int cnt = 1;

  readFormat();

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
    }
    pub_poseArray.publish(frontEnd.get_poseArray()); // 姿勢描画

    cnt++;
  }

  return;
}
