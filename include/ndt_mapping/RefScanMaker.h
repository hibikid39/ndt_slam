#ifndef _REF_SCAN_MAKER_H_
#define _REF_SCAN_MAKER_H_

#include <vector>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"
#include "PointCloudMap.h"

class RefScanMaker {
private:
  const PointCloudMap *pcmap;           // 点群地図
  Scan2D refScan;                       // 参照スキャン本体。これを外に提供

public:
  RefScanMaker() : pcmap(NULL) {
  }

  ~RefScanMaker() {
  }

  void setPointCloudMap(const PointCloudMap *pcmap_) {
    pcmap = pcmap_;
  }

  Scan2D *makeRefScan();

};

#endif
