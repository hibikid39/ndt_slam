#ifndef SCAN_POINT_RESAMPLER_H_
#define SCAN_POINT_RESAMPLER_H_

#include <ros/ros.h>
#include <vector>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Scan2D.h"


class ScanPointResampler {
private:
  double space;                      // 点の距離間隔[m]
  double spaceThre;                  // 点の距離閾値[m]。この間隔を超えたら補間しない
  double dis;                        // 累積距離。作業用　distance

public:
  ScanPointResampler() : space(0.0), spaceThre(0.0), dis(0) {
    ros::param::get("space", space);
    ros::param::get("space_thre" ,spaceThre);
  }

  ~ScanPointResampler() {
  }

///////

  void setDthre(int s, int l) {
    space = s;
    spaceThre = l;
  }

////////

  void resamplePoints(Scan2D *scan);
  bool findInterpolatePoint(const LPoint2D &cp, const LPoint2D &pp, LPoint2D &np, bool &inserted);
};

#endif
