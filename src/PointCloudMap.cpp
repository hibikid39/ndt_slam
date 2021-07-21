#include "ndt_mapping/PointCloudMap.h"
using namespace std;

////// 部分地図 ///////

// 格子テーブルを用いて、部分地図の代表点を得る
vector<LPoint2D> Submap::subsamplePoints(int nthre, NNGridTable *nntab) {
  double x_bottom = 10000000000;
  double x_top = -10000000000;
  double y_bottom = 10000000000;
  double y_top = -10000000000;

  for (size_t i=0; i<lps.size(); i++) {
    LPoint2D &lp = lps[i];
    if(false == (nntab->addPoint(&lp))) {     // NNGridTableに登録できたらfalse
      if(x_bottom > lp.x) x_bottom = lp.x;
      if(x_top < lp.x) x_top = lp.x;
      if(y_bottom > lp.y) y_bottom = lp.y;
      if(y_top < lp.y) y_top = lp.y;
    }
  }

  ROS_INFO("%f, %f, %f, %f", x_bottom, x_top, y_bottom, y_top);

  // nthre個以上のセルの代表点をspsに入れる
  vector<LPoint2D> sps;
  nntab->makeCellPoints(nthre, sps, x_bottom, x_top, y_bottom, y_top);
  ROS_INFO("[Submap::subsamplePoints] lps.size=%lu, sps.size=%lu", lps.size(), sps.size());

  nntab->clear(x_bottom, x_top, y_bottom, y_top);

  return(sps);
}

///////////////////////

// ロボット位置の追加
void PointCloudMap::addPose(const Pose2D &p) {
  // 累積走行距離(atd)の計算
  if (poses.size() > 0) {
    Pose2D pp = poses.back(); // 一つ前(posesの末尾)のロボット位置
    atd += sqrt((p.tx - pp.tx)*(p.tx - pp.tx) + (p.ty - pp.ty)*(p.ty - pp.ty));
  }
  else {
    atd += sqrt(p.tx*p.tx + p.ty*p.ty);
  }

  poses.push_back(p);
}

// スキャン点群の追加
void PointCloudMap::addPoints(const std::vector<LPoint2D> &lps) {
  Submap &curSubmap = submaps.back();              // 現在の部分地図
  if (atd - curSubmap.atdS >= sepThre ) {          // 累積走行距離が閾値を超えたら新しい部分地図に変える
    size_t size = poses.size();                    // posesにはすでに最新値を追加済みなので-1
    curSubmap.cntE = size-1;                       // 部分地図の最後のスキャン番号
    curSubmap.lps = curSubmap.subsamplePoints(nthre, nntab); // 終了した部分地図は代表点のみにする（軽量化）

/*
    // 終了したサブマップの代表自己位置を保存
    double x = 0;
    double y = 0;
    double th = 0;
    for(int i=curSubmap.cntS; i<curSubmap.cntE; i++) {
      x += poses[i].tx;
      y += poses[i].ty;
      th += poses[i].th; // 正規化しない
    }
    curSubmap.repPose.tx = x/(curSubmap.cntE - curSubmap.cntS);
    curSubmap.repPose.ty = y/(curSubmap.cntE - curSubmap.cntS);
    curSubmap.repPose.th = MyUtil::add_angle(th/(curSubmap.cntE - curSubmap.cntS), 0); // 0足して正規化
*/

    Submap submap(atd, size);                      // 新しい部分地図
    submap.addPoints(lps);                         // スキャン点群の登録
    submaps.emplace_back(submap);                  // 部分地図を追加
  } else {                                         // 超えていなければ
    curSubmap.addPoints(lps);                      // 現在の部分地図に点群を追加
  }
}

///////////

// 全体地図生成 局所地図もここで生成
void PointCloudMap::makeGlobalMap(){
  geometry_msgs::Point32 xyz;

  pcmap_ros.points.clear();
  pcmap_ros.channels[0].values.clear();

//  timer.start_timer();

  globalMap.clear();                               // 初期化
//  localMap.clear();

  // 現在以外のすでに確定した部分地図から点を集める
  for (size_t i=0; i<submaps.size()-1; i++) {
    Submap &submap = submaps[i];                   // 部分地図
    vector<LPoint2D> &lps = submap.lps;            // 部分地図の点群 代表点だけになっている
    for (size_t j=0; j<lps.size(); j++) {
      globalMap.emplace_back(lps[j]);              // 全体地図には全点入れる

      // ROS用
      xyz.x = lps[j].x;
      xyz.y = lps[j].y;
      xyz.z = 0.0;
      pcmap_ros.points.emplace_back(xyz);
      uint32_t r = colorList[i%colorList.size()][0];
      uint32_t g = colorList[i%colorList.size()][1];
      uint32_t b = colorList[i%colorList.size()][2];
      uint32_t rgb = (r << 16 | g << 8 | b);
      pcmap_ros.channels[0].values.emplace_back(*reinterpret_cast<float*>(&rgb));   // (R,G,B) = (255, 255, 255)
    }
/*
    if (i == submaps.size()-2) {                   // 局所地図には最後の部分地図だけ入れる
      for (size_t j=0; j<lps.size(); j++) {
        localMap.emplace_back(lps[j]);
      }
    }
*/
  }

  // 現在の部分地図の代表点を全体地図と局所地図に入れる
  Submap &curSubmap = submaps.back();              // 現在の部分地図
  vector<LPoint2D> sps = curSubmap.subsamplePoints(nthre, nntab);  // 代表点を得る
//  vector<LPoint2D> sps = curSubmap.lps;
  for (size_t i=0; i<sps.size(); i++) {
    globalMap.emplace_back(sps[i]);
//    localMap.emplace_back(sps[i]);

    // ROS用
    xyz.x = sps[i].x;
    xyz.y = sps[i].y;
    xyz.z = 0.0;
    pcmap_ros.points.emplace_back(xyz);
    uint32_t r = 255, g = 255, b = 255;
    uint32_t rgb = (r << 16 | g << 8 | b);
    pcmap_ros.channels[0].values.emplace_back(*reinterpret_cast<float*>(&rgb));   // (R,G,B) = (255, 255, 255)

  }

  // 以下は確認用
  ROS_INFO("[PointCloudMap::makeGlobalMap] curSubmap.atd=%g, atd=%g, sps.size=%lu", curSubmap.atdS, atd, sps.size());
  ROS_INFO("[PointCloudMap::makeGlobalMap] submaps.size=%lu, globalMap.size=%lu", submaps.size(), globalMap.size());

//  timer.end_timer();
//  timer.print_timer();
}

// 局所地図生成
void PointCloudMap::makeLocalMap(){
  localMap.clear();                                // 初期化

  // 最新の地図だけではデータ数が少ないので 直前の部分地図も使う
  if (submaps.size() >= 2) {
    Submap &submap = submaps[submaps.size()-2];    // 直前の部分地図だけ使う
    vector<LPoint2D> &lps_old = submap.lps;            // 部分地図の点群 代表点だけになっている
    for (size_t i=0; i<lps_old.size(); i++) {
      localMap.emplace_back(lps_old[i]);
    }

/*
    // 以前に訪れたことのある部分地図を探索して局所地図に入れる
    double dst;
    double dstMin = 10000000000;
    size_t min_i = 0;
    for (size_t i=0; i<submaps.size()-2; i++) {
      dst = (lastPose.tx - submaps[i].repPose.tx)*(lastPose.tx - submaps[i].repPose.tx) + (lastPose.ty - submaps[i].repPose.ty)*(lastPose.ty - submaps[i].repPose.ty);
      if (dst < dstMin) {
        dstMin = dst;
        min_i = i;
      }
    }
    vector<LPoint2D> &lps_revisit = submaps[min_i].lps;
    for (size_t i=0; i<lps_revisit.size(); i++) {
      localMap.emplace_back(lps_revisit[i]);
    }
*/

  }

  // 現在の部分地図の代表点を局所地図に入れる
  Submap &curSubmap = submaps.back();              // 現在の部分地図
  vector<LPoint2D> lps_cur = curSubmap.subsamplePoints(nthre, nntab);  // 代表点を得る
//  vector<LPoint2D> lps_cur = curSubmap.lps;
  for (size_t i=0; i<lps_cur.size(); i++) {
    localMap.emplace_back(lps_cur[i]);
  }

  ROS_INFO("[PointCloudMap::makeLocalMap] localMap.size=%lu", localMap.size());   // 確認用
}

// ポーズ調整後のロボット軌跡newPoseを用いて、地図を再構築する
void PointCloudMap::remakeMaps(const std::vector<Pose2D> &newPoses) {
  // 各部分地図内の点の位置を修正する
  for (size_t i=0; i<submaps.size(); i++) {
    Submap &submap = submaps[i];
    vector<LPoint2D> &mps = submap.lps;                // 部分地図の点群。現在地図以外は代表点になっている
    for (size_t j=0; j<mps.size(); j++) {
      LPoint2D &mp = mps[j];
      size_t idx = mp.sid - startFrame;                          // 点のスキャン番号

      const Pose2D &oldPose = poses[idx];              // mpに対応する古いロボット位置
      const Pose2D &newPose = newPoses[idx];           // mpに対応する新しいロボット位置
      const double (*R1)[2] = oldPose.Rmat;
      const double (*R2)[2] = newPose.Rmat;
      LPoint2D lp1 = oldPose.relativePoint(mp);        // oldPoseでmpをセンサ座標系に変換
      LPoint2D lp2 = newPose.globalPoint(lp1);         // newPoseでポーズ調整後の地図座標系に変換
      mp.x = lp2.x;
      mp.y = lp2.y;
      double nx = R1[0][0]*mp.nx + R1[1][0]*mp.ny;     // 法線ベクトルもoldPoseでセンサ座標系に変換
      double ny = R1[0][1]*mp.nx + R1[1][1]*mp.ny;
      double nx2 = R2[0][0]*nx + R2[0][1]*ny;          // 法線ベクトルもnewPoseでポーズ調整後の地図座標系に変換
      double ny2 = R2[1][0]*nx + R2[1][1]*ny;
      mp.setNormal(nx2, ny2);
    }
  }

  makeGlobalMap();                                     // 部分地図から全体地図と局所地図を生成
  makeLocalMap();

  for (size_t i=0; i<poses.size(); i++) {              // posesをポーズ調整後の値に更新
    poses[i] = newPoses[i];
  }
  lastPose = newPoses.back();
}