#ifndef SCAN_ANALYSER_H_
#define SCAN_ANALYSER_H_

#include <vector>

#include "MyUtil.h"
#include "LPoint2D.h"
#include "Scan2D.h"

class ScanPointAnalyser {
private:
  static const double FPDMIN;         // 隣接点との最小距離[m] これより小さいと誤差が大きくなるので法線計算に使わない
  static const double FPDMAX;         // 隣接点との最大距離[m] これより大きいと不連続とみなして法線計算に使わない
  static const int CRTHRE = 45;       // 法線方向変化の閾値[度] これより大きいとコーナ点とみなす
  static const int INVALID = -1;
  double costh;                       // 左右の法線方向の食い違いの閾値

public:
  ScanPointAnalyser() : costh(cos(DEG2RAD(CRTHRE))) {
  }

  ~ScanPointAnalyser() {
  }

//////////

  void analysePoints(std::vector<LPoint2D> &lps);
  bool calNormal(int idx, const std::vector<LPoint2D> &lps, int dir, Vector2D &normal);

};

#endif
