#include "ndt_mapping/NNGridTable.h"

// 格子テーブルにスキャン点lpを登録する true:削除, false削除しない
bool NNGridTable::addPoint(const LPoint2D *lp) {
  // テーブル検索のインデックス計算。まず、対象領域内にあるかチェックする
  int xi = static_cast<int>(lp->x/csize) + tsize;  // (i,j)=(tsize,tsize)が原点
  if (xi < 0 || xi > 2*tsize)                      // 対象領域の外
    return true;
  int yi = static_cast<int>(lp->y/csize) + tsize;
  if (yi < 0 || yi > 2*tsize)                      // 対象領域の外
    return true;

  size_t idx = static_cast<size_t>(yi*(2*tsize +1) + xi);   // テーブルのインデックス
  table[idx].lps.push_back(lp);                             // 目的のセルに入れる

  return false;
//  if (table[idx].lps.size() >= 8) return true;  // 格子に4点以上ある場合 lpは消去
//  else return false;
}

// 格子テーブルの各セルの代表点を作ってpsに格納する
void NNGridTable::makeCellPoints(int nthre, std::vector<LPoint2D> &globalMap,
  double x_bottom, double x_top, double y_bottom, double y_top) {
//  size_t nn=0;                    // テーブル内の全セル数 確認用

  int xi_bottom = static_cast<int>(x_bottom/csize) + tsize;
  int xi_top = static_cast<int>(x_top/csize) + tsize;
  int yi_bottom = static_cast<int>(y_bottom/csize) + tsize;
  int yi_top = static_cast<int>(y_top/csize) + tsize;

  for (int xi=xi_bottom; xi<=xi_top; xi++) {
    for (int yi=yi_bottom; yi<=yi_top; yi++) {
      size_t idx = static_cast<size_t>(yi*(2*tsize +1) + xi);   // テーブルのインデックス

      std::vector<const LPoint2D*> &lps = table[idx].lps;      // セルのスキャン点群
  //    nn += lps.size();                  // 確認用
      if (lps.size() >= nthre) {           // 点数がnthreより多いセルだけ処理する
        double gx=0, gy=0;                 // 点群の重心位置
        double nx=0, ny=0;                 // 点群の法線ベクトルの平均
        int sid=0;
        for (size_t j=0; j<lps.size(); j++) {
          const LPoint2D *lp = lps[j];
          gx += lp->x;                     // 位置を累積
          gy += lp->y;
          nx += lp->nx;                    // 法線ベクトル成分を累積
          ny += lp->ny;
          //sid += lp->sid;                  // スキャン番号の平均をとる
          if (lp->sid > sid)             // スキャン番号の最新値とる場合
            sid = lp->sid;
        }
        gx /= lps.size();                  // 平均
        gy /= lps.size();
        double L = std::sqrt(nx*nx + ny*ny);
        nx /=  L;                          // 平均（正規化）
        ny /=  L;
        //sid /= lps.size();                 // スキャン番号の平均をとる

        LPoint2D newLp(sid, gx, gy);       // セルの代表点を生成
        newLp.setNormal(nx, ny);           // 法線ベクトル設定
        newLp.setType(LINE);               // タイプは直線にする
        globalMap.push_back(newLp);            // psに追加
      }
    }
  }

/*
  for (size_t i=0; i<table.size(); i++) {
    std::vector<const LPoint2D*> &lps = table[i].lps;      // セルのスキャン点群
//    nn += lps.size();                  // 確認用
    if (lps.size() >= nthre) {           // 点数がnthreより多いセルだけ処理する
      double gx=0, gy=0;                 // 点群の重心位置
      double nx=0, ny=0;                 // 点群の法線ベクトルの平均
      int sid=0;
      for (size_t j=0; j<lps.size(); j++) {
        const LPoint2D *lp = lps[j];
        gx += lp->x;                     // 位置を累積
        gy += lp->y;
        nx += lp->nx;                    // 法線ベクトル成分を累積
        ny += lp->ny;
        //sid += lp->sid;                  // スキャン番号の平均をとる
        if (lp->sid > sid)             // スキャン番号の最新値とる場合
          sid = lp->sid;
      }
      gx /= lps.size();                  // 平均
      gy /= lps.size();
      double L = std::sqrt(nx*nx + ny*ny);
      nx /=  L;                          // 平均（正規化）
      ny /=  L;
      //sid /= lps.size();                 // スキャン番号の平均をとる

      LPoint2D newLp(sid, gx, gy);       // セルの代表点を生成
      newLp.setNormal(nx, ny);           // 法線ベクトル設定
      newLp.setType(LINE);               // タイプは直線にする
      globalMap.push_back(newLp);            // psに追加
    }
  }
*/

//  ROS_INFO("nn=%d", nn);               // テーブル内の全セル数。確認用
}

// スキャン点clpをpredPoseで座標変換した位置に最も近い点を格子テーブルから見つける
const LPoint2D *NNGridTable::findClosestPoint(const LPoint2D *clp, const Pose2D &predPose) {
  LPoint2D glp;                           // clpの予測位置
  predPose.globalPoint(*clp, glp);         // relPoseで座標変換

  // clpのテーブルインデックス 対象領域内にあるかチェックする (x,y)->(i,j)
  int cxi = static_cast<int>(glp.x/csize) + tsize;
  if (cxi < 0 || cxi > 2*tsize)
    return(nullptr);
  int cyi = static_cast<int>(glp.y/csize) + tsize;
  if (cyi < 0 || cyi > 2*tsize)
    return(nullptr);

  size_t pn=0;                            // 探したセル内の点の総数 確認用
  double dmin=1000000;
  const LPoint2D *lpmin = nullptr;        // 最も近い点（目的の点）
  double dthre=0.24;                      // これより遠い点は除外する[m] > ロボットの速度
  int R=static_cast<int>(dthre/csize);    // 近傍とみなす範囲のインデクス

  // ±R四方を探す
  for (int i=-R; i<=R; i++) {
    int yi = cyi+i;                       // cyiから広げる
    if (yi < 0 || yi > 2*tsize)           // テーブル外ならとばす
      continue;
    for (int j=-R; j<=R; j++) {
      int xi = cxi+j;                     // cxiから広げる
      if (xi < 0 || xi > 2*tsize)         // テーブル外ならとばす
        continue;

      size_t idx = yi*(2*tsize+1) + xi;             // テーブルインデックス
      NNGridCell &cell = table[idx];                // そのセル
      std::vector<const LPoint2D*> &lps = cell.lps;      // セルがもつスキャン点群
      for (size_t k=0; k<lps.size(); k++) {
        const LPoint2D *lp = lps[k];
        double d = (lp->x - glp.x)*(lp->x - glp.x) + (lp->y - glp.y)*(lp->y - glp.y); // 点間距離

        if ((d <= dthre*dthre) && (d < dmin)) {         // dthre内で距離が最小となる点を保存
          dmin = d;
          lpmin = lp;
        }
      }
      pn += lps.size();
    }
  }
//  printf("pn=%d\n", pn);                 // 探したセル内の点の総数 確認用

  return(lpmin);
}
