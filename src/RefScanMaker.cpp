#include "ndt_mapping/RefScanMaker.h"

// 参照スキャンを作成 局所地図(現在は全体地図)を参照スキャンに設定
// 戻り値 : 参照スキャンのポインタ
Scan2D *RefScanMaker::makeRefScan() {
  std::vector<LPoint2D> &refLps = refScan.lps;         // 参照スキャンの点群のコンテナ
  refLps.clear(); // vectorの全要素削除

  Pose2D lastPose = pcmap->getLastPose();              // 点群地図に保存した最後の推定位置

  // 点群地図に保存した最後のスキャンを参照スキャンにする
  const std::vector<LPoint2D> &localMap = pcmap->localMap;  // 局所地図(現在は全体地図)
  for (size_t i=0; i<localMap.size(); i++) {
    refLps.push_back(localMap[i]);                     // loclaMapはすでにワールド座標系
  }

  return(&refScan);
}
