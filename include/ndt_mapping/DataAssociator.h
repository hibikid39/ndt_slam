#ifndef DATA_ASSOCIATOR_H_
#define DATA_ASSOCIATOR_H_

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "NNGridTable.h"

class DataAssociator{
public:
  std::vector<const LPoint2D*> curLps;            // 対応がとれた現在スキャンの点群
  std::vector<const LPoint2D*> refLps;            // 対応がとれた参照スキャンの点群
  std::vector<const LPoint2D*> baseLps;           // 参照スキャンの点を格納しておく 作業用
  NNGridTable *baseNntab;  // 作業用 格子テーブル

  double x_bottom, x_top, y_bottom, y_top; // baseNntabをclearするために点のある領域を保存しておく

  DataAssociator() {
  }

  ~DataAssociator() {
  }

////////////////////////////////////////////////////////////////////////////////

  void setNNGridTable(NNGridTable *nntab_) {
    baseNntab = nntab_;
  }

  void clearNntab() {
    baseNntab->clear(x_bottom, x_top, y_bottom, y_top);
  }

////////////////////////////////////////////////////////////////////////////////

  void setRefBase(const std::vector<LPoint2D> &lps);
  double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose);
};

#endif
