#include "ndt_mapping/PoseOptimizer.h"

// データ対応づけ固定のもと、初期値initPoseを与えてロボット位置の推定値estPoseを求める
double PoseOptimizer::optimizePose(Pose2D &initPose, Pose2D &estPose) {
  double th = initPose.th;
  double tx = initPose.tx;
  double ty = initPose.ty;
  double txmin=tx, tymin=ty, thmin=th;         // コスト最小の解
  double costMin = HUGE_VAL;                     // コストの最小値
  double costOld = costMin;                        // 1つ前のコスト値。収束判定に使う
  Pose2D pose, dir;

  double cost = cfunc->calValue(tx, ty, th);   // コスト計算
  int nn=0;                                    // 繰り返し回数。確認用
//  double kk=0.00001;                           // 最急降下法のステップ幅係数
  while (std::abs(costOld-cost) > thre) {             // 収束判定。1つ前の値との変化が小さいと終了
    nn++;
    costOld = cost;

    // 数値計算による偏微分
    double dx = (cfunc->calValue(tx+dd, ty, th) - cost)/dd;
    double dy = (cfunc->calValue(tx, ty+dd, th) - cost)/dd;
    double dth = (cfunc->calValue(tx, ty, th+da) - cost)/da;

    // ブレント法による直線探索
    pose.tx = tx;  pose.ty = ty;  pose.th = th;   // 探索開始点
    dir.tx = dx;   dir.ty = dy;   dir.th = dth;   // 探索方向
    search(pose, dir);                        // 直線探索実行
    tx = pose.tx;  ty = pose.ty;  th = pose.th;   // 直線探索で求めた位置

    cost = cfunc->calValue(tx, ty, th);         // その位置でコスト計算

    if (cost < costMin) {                       // costがこれまでの最小なら更新
      costMin = cost;
      txmin = tx;  tymin = ty;  thmin = th;
    }

//    printf("[PoseOptimizer] nn=%d, ev=%g, evold=%g, abs(evold-ev)=%g\n", nn, ev, evold, abs(evold-ev));         // 確認用
  }

  ++allN; // 繰り返し回数
  if (allN > 0 && costMin < 100)  // 100 ?
    sum += costMin;

//  printf("[PoseOptimizer] allN=%d, costMin=%g, avg=%g\n", allN, costMin, (sum/allN));         // 確認用

//  printf("[PoseOptimizer] nn=%d, cost=%g\n", nn, cost);         // 確認用

  estPose.setPose(txmin, tymin, thmin);          // 最小値を与える解を保存

  return(costMin);
}

// boostライブラリのブレント法で直線探索を行う。
// poseを始点に、dp方向にどれだけ進めばよいかステップ幅を見つける。
double PoseOptimizer::search(Pose2D &pose, Pose2D &dp) {
  int bits = std::numeric_limits<double>::digits;         // 探索精度
  boost::uintmax_t maxIter=40;                       // 最大繰り返し回数。経験的に決める
  std::pair<double, double> result =
    boost::math::tools::brent_find_minima(
    [this, &pose, &dp](double tt) {return (objFunc(tt, pose, dp));},  // objFuncが最小となるttを求める tt=ステップ幅
    -2.0, 2.0, bits, maxIter);                       // 探索範囲(-2.0,2.0)

  double t = result.first;                           // 求めるステップ幅
  double v = result.second;                          // 求める最小値

  pose.tx = pose.tx + t*dp.tx;                       // 求める最小解をposeに格納
  pose.ty = pose.ty + t*dp.ty;
  pose.th = MyUtil::add_angle(pose.th, t*dp.th);

  return(v);
}

// 直線探索の目的関数。ttがステップ幅
double PoseOptimizer::objFunc(double tt, Pose2D &pose, Pose2D &dp) {
  double tx = pose.tx + tt*dp.tx;                     // poseからdp方向にttだけ進む
  double ty = pose.ty + tt*dp.ty;
  double th = MyUtil::add_angle(pose.th, tt*dp.th);
  double v = cfunc->calValue(tx, ty, th);             // コスト関数値

  return(v);
}
