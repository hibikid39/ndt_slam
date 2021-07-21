#include "ndt_mapping/ScanPointResampler.h"

// scanを均一化
void ScanPointResampler::resamplePoints(Scan2D *scan) {
  int c=0;
  std::vector<LPoint2D> &lps = scan->lps;           // スキャン点群
  if (lps.size() == 0)
    return;

  std::vector<LPoint2D> newLps;             // リサンプル後の点群

  dis = 0;                                  // disは累積距離
  LPoint2D lp = lps[0];
  LPoint2D prevLp = lp;
  LPoint2D np(lp.sid, lp.x, lp.y);
  newLps.push_back(np);                     // 最初の点は入れる
  for (size_t i=1; i<lps.size(); i++) {
    lp = lps[i];                            // スキャン点
    bool inserted=false;

    bool exist = findInterpolatePoint(lp, prevLp, np, inserted);

    if (exist) {                               // 入れる点がある
      c++;
      newLps.push_back(np);                    // 新しい点npを入れる
      prevLp = np;                             // npが直前点になる
      dis = 0;                                 // 累積距離をリセット
      if (inserted) i--;                       // lpの前で補間点を入れたので、lpをもう一度やる
    } else {
      prevLp = lp;                             // 今のlpが直前点になる
    }
  }

//  printf("[ScanPointResampler] lps.size=%lu, newLps.size=%lu\n", lps.size(), newLps.size());    // 確認用

  scan->setLps(newLps);
}

// 現在の点cp,1つ前の点pp,累計距離disから点を打つか判断し,打つ場合npに座標を設定
// cp:currentPoint, pp:previousPoint, np:newPoint
bool ScanPointResampler::findInterpolatePoint(const LPoint2D &cp, const LPoint2D &pp, LPoint2D &np, bool &inserted) {
  double dx = cp.x - pp.x;
  double dy = cp.y - pp.y;
  double L = sqrt(dx*dx+dy*dy);             // 現在点cpと直前点ppの距離

//  printf("[ADJUST][ScanPointResampler] Point interval : %f\n", L);

  if (dis+L < space) {                     // 失敗 予測累積距離(dis+L)がspaceより小さい点は削除
    dis += L;                               // disに加算
    return(false);
  } else if (dis+L >= spaceThre) {               // 成功予測累積距離がspaceThreより大きい点は補間せず、そのまま残す
    np.setData(cp.sid, cp.x, cp.y);
  } else {                                    // 予測累積距離がspaceを超えたら、spaceになるように補間する
    double ratio = (space-dis)/L;
    double x2 = dx*ratio + pp.x;            // 少し伸ばして距離がspaceになる位置
    double y2 = dy*ratio + pp.y;
    np.setData(cp.sid, x2, y2);
    inserted = true;                        // cpより前にnpを入れたというフラグ
  }

  return(true);
}
