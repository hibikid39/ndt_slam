#include "ndt_mapping/CostFunction.h"

// 垂直距離による推測自己位置(tx,ty,th)でのコスト関数
// 誤差の二乗を100倍した値の合計を返す
double CostFunction::calValue(double tx, double ty, double th) {
  double rad = DEG2RAD(th);
  double error=0; // 各点の誤差を累積
  int pn=0;       // 誤差が小さい点の数
  int nn=0;       // 比較した点の数 有効点数

  for (size_t i=0; i<curLps.size(); i++) {
    const LPoint2D *clp = curLps[i];             // 現在スキャンの点
    const LPoint2D *rlp = refLps[i];             // clpに対応する参照スキャンの点

    if(rlp->type != LINE) continue;

    double cx = clp->x;
    double cy = clp->y;
    // clpを参照スキャンの座標系に変換 ワールド座標系
    double x = cos(rad)*cx - sin(rad)*cy + tx;
    double y = sin(rad)*cx + cos(rad)*cy + ty;

    double edis = (x - rlp->x)*(rlp->nx) + (y - rlp->y)*(rlp->ny);     // 垂直距離
    double er = edis*edis;  // 誤差の二乗で検証するため

    if (er <= limit*limit)
      ++pn;                  // 誤差が小さい点の数

    error += er;           // 各点の誤差を累積
    ++nn;                    // 比較した点の数
  }

  error = (nn>0)? (error + 0.0000001)/nn : HUGE_VAL;           // 平均をとる。有効点数が0なら、値はHUGE_VAL
  pnrate = 1.0*(pn + 0.0000001)/nn;                            // 誤差が小さい点の比率
  error *= 100;                                  // 評価値が小さくなりすぎないよう100かける。

  return(error);
}
