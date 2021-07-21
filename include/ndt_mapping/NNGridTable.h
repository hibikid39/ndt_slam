#ifndef NN_GRID_TABLE_H_
#define NN_GRID_TABLE_H_

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"
#include "Timer.h"

struct NNGridCell {
  std::vector<const LPoint2D*> lps;         // このセルに格納されたスキャン点群 ポインタ

  void clear() {
    lps.clear();                            // 空にする
  }
};

///////////////////////////////////

// 格子テーブル
class NNGridTable {
private:
  double csize;                       // セルサイズ[m]
  double rsize;                       // 対象領域のサイズ[m] 正方形の1辺の半分 radiusSize
  int tsize;                          // テーブルサイズの半分(1辺の要素数の半分)
  std::vector<NNGridCell> table;      // テーブル本体

  Timer timer;

public:
//  nav_msgs::OccupancyGrid ocg;            // ROSメッセージ用占有格子地図

  NNGridTable() : csize(0.0), rsize(0.0){          // セル5cm 対象領域10x2m四方
    ros::param::get("cell_size", csize);
    ros::param::get("radius_size", rsize);

    tsize = static_cast<int>(rsize/csize);           // テーブルサイズの半分
    size_t w = static_cast<int>(2*tsize + 1);          // テーブルサイズ
    table.resize(w*w);                               // 領域確保
//    clear();                                         // tableの初期化

/*
     // ocg領域確保, info入力
    ocg.data.resize(w*w);
    for(int i = 0; i < w*w; i++){
      ocg.data[i] = -1; // -1 is unknown
    }
    ocg.info.resolution = csize;
    ocg.info.width = static_cast<int>(2*tsize + 1);
    ocg.info.height = static_cast<int>(2*tsize + 1);
    ocg.info.origin.position.x = -1.0*rsize;
    ocg.info.origin.position.y = -1.0*rsize;
    ocg.info.origin.position.z = 0;
    ocg.info.origin.orientation = MyUtil::rpy_to_geometry_quat(0, 0, 0);
*/
  }

  ~NNGridTable() {
  }

  // 全領域をクリア
  void clear() {
    timer.start_timer();
    for (size_t i=0; i<table.size(); i++)
      table[i].clear();                         // 各セルを空にする
    timer.end_timer();
    timer.print_timer();
  }

  // 指定範囲をクリア
  void clear(double x_bottom, double x_top, double y_bottom, double y_top) {
    int xi_bottom_ = static_cast<int>(x_bottom/csize) + tsize;
    int xi_top_ = static_cast<int>(x_top/csize) + tsize;
    int yi_bottom_ = static_cast<int>(y_bottom/csize) + tsize;
    int yi_top_ = static_cast<int>(y_top/csize) + tsize;

    for (int xi=xi_bottom_; xi<=xi_top_; xi++) {
      for (int yi=yi_bottom_; yi<=yi_top_; yi++) {
        size_t idx = static_cast<size_t>(yi*(2*tsize +1) + xi);   // テーブルのインデックス
        table[idx].clear();   // 各セルを空にする
      }
    }
  }

////////////

  bool addPoint(const LPoint2D *lp);
  void makeCellPoints(int nthre, std::vector<LPoint2D> &globalMap,
      double x_bottom, double x_top, double y_bottom, double y_top);
  const LPoint2D *findClosestPoint(const LPoint2D *clp, const Pose2D &predPose);

};

#endif
