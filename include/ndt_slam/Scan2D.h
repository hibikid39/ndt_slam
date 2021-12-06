#ifndef SCAN2D_H_
#define SCAN2D_H_

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <vector>

#include "Pose2D.h"
#include "LPoint2D.h"


//////////

// スキャン
struct Scan2D {
  int sid;                                   // スキャンid
  Pose2D pose;                               // スキャン取得時のオドメトリ値
  std::vector<LPoint2D> lps;                 // スキャン点群

  std_msgs::Header header;

  Scan2D() : sid(0) {
  }
  ~Scan2D() {
  }

//////////

  void setLps (const std::vector<LPoint2D> &ps) {
    lps = ps;
  }



};

#endif
