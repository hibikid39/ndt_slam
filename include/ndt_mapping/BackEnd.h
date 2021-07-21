#ifndef SLAM_BACK_END_H_
#define SLAM_BACK_END_H_

#include <vector>
#include "PointCloudMap.h"
#include "PoseGraph.h"

////////

class BackEnd {
public:
  std::vector<Pose2D> newPoses;            // ポーズ調整後の姿勢
  PointCloudMap *pcmap;                    // 点群地図
  PoseGraph *pg;                           // ポーズグラフ

  BackEnd() {
  }

  ~BackEnd() {
  }

////////////////////////////////////////////////////////////////////////////////

  void setPointCloudMap(PointCloudMap *pcmap_) {
    pcmap = pcmap_;
  }

  void setPoseGraph(PoseGraph *pg_) {
    pg = pg_;
  }

////////////////////////////////////////////////////////////////////////////////

  Pose2D adjustPoses();
  void remakeMaps();
};

#endif
