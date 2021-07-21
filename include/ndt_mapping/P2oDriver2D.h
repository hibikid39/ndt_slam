#ifndef P2O_DRIVER2D_H_
#define P2O_DRIVER2D_H_

#include <iostream>
#include <fstream>
#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PoseGraph.h"

//////////

// ポーズグラフ最適化ライブラリkslamを起動する。
class P2oDriver2D
{
public:

  P2oDriver2D() {
  }

  ~P2oDriver2D() {
  }

///////

  void doP2o( PoseGraph &graph, std::vector<Pose2D> &newPoses, int N);

};

#endif
