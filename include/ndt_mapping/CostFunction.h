#ifndef COST_FUNCTION_H_
#define COST_FUNCTION_H_

#include <vector>
#include "MyUtil.h"
#include "LPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"

class CostFunction{
private:
  std::vector<const LPoint2D*> curLps;         // 対応がとれた現在スキャンの点群
  std::vector<const LPoint2D*> refLps;         // 対応がとれた参照スキャンの点群
  double limit;                                // マッチングで対応がとれたと見なす距離閾値
  double pnrate;                               // 誤差がcostLimit以内で対応がとれた点の比率

public:
  CostFunction() : limit(0), pnrate(0) {}
  ~CostFunction() {}

///////

  void setLimit(double e) {
    limit = e;
  }

  // DataAssociatorで対応のとれた点群cur, refを設定
  void setPoints(std::vector<const LPoint2D*> &cur, std::vector<const LPoint2D*> &ref) {
    curLps = cur;
    refLps = ref;
  }

  double getPnrate() {
    return(pnrate);
  }

///////////

  double calValue(double tx, double ty, double th);

};

#endif
