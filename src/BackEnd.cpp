#include "ndt_mapping/BackEnd.h"
#include "ndt_mapping/P2oDriver2D.h"

using namespace std;

////////// ポーズ調整 //////////

Pose2D BackEnd::adjustPoses() {
//  pg->printArcs();
//  pg->printNodes();

  newPoses.clear();

  P2oDriver2D p2o;
  p2o.doP2o(*pg, newPoses, 5);                 // 5回くり返す

  return(newPoses.back());
}

/////////////////////////////

void BackEnd::remakeMaps() {
  // PoseGraphの修正
  vector<PoseNode*> &pnodes = pg->nodes;      // ポーズノード
  for (size_t i=0; i<newPoses.size(); i++) {
    Pose2D &npose = newPoses[i];
    PoseNode *pnode = pnodes[i];              // ノードはロボット位置と1:1対応
    pnode->setPose(npose);                    // 各ノードの位置を更新
  }
  ROS_INFO("[BackEnd::remakeMaps] newPoses.size=%lu, nodes.size=%lu", newPoses.size(), pnodes.size());

  // PointCloudMapの修正
  pcmap->remakeMaps(newPoses);
}
