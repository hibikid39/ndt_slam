//#include <boost/timer.hpp>
#include "ndt_mapping/DataAssociator.h"

// 参照スキャンをbaseNntabに格納しておく 地図の端を保存しておく 使い終わったら必ずclearNntabで消去
void DataAssociator::setRefBase(const std::vector<LPoint2D> &rlps){
//  baseNntab->clear();
  x_bottom = 10000000000;
  x_top = -10000000000;
  y_bottom = 10000000000;
  y_top = -10000000000;

  for (size_t i=0; i<rlps.size(); i++) {
    const LPoint2D &lp = rlps[i];
    if(false == baseNntab->addPoint(&lp)) {     // NNGridTableに登録できたらfalse
      if(x_bottom > lp.x) x_bottom = lp.x;
      if(x_top < lp.x) x_top = lp.x;
      if(y_bottom > lp.y) y_bottom = lp.y;
      if(y_top < lp.y) y_top = lp.y;
    }
  }
}

// 現在スキャンcurScanの各スキャン点に対応する点をbaseLpsから見つける
// curScan:現在スキャン, predPose:暫定ポーズ
double DataAssociator::findCorrespondence(const Scan2D *curScan, const Pose2D &predPose) {
  curLps.clear();                                   // 対応づけ現在スキャン点群を空にする
  refLps.clear();                                   // 対応づけ参照スキャン点群を空にする

  for (size_t i=0; i<curScan->lps.size(); i++) {
    const LPoint2D *clp = &(curScan->lps[i]);       // 現在スキャンの点 ポインタ

    // 格子テーブルから最近傍点を求める
    const LPoint2D *rlp = baseNntab->findClosestPoint(clp, predPose);

    if (rlp != NULL) {                        // 最近傍点があれば登録
      curLps.push_back(clp);
      refLps.push_back(rlp);
    }
  }

  double ratio = (1.0*curLps.size())/curScan->lps.size();         // 対応がとれた点の比率
//  printf("ratio=%g, clps.size=%lu\n", ratio, curScan->lps.size());

  return(ratio);
}
