#include "ndt_mapping/Pose2D.h"

// prevPoseからcurPoseまでの相対位置(移動距離)を求める -> Motion
// Motionはロボット座標系での移動量 dx,dyはワールド座標系
void Pose2D::calMotion(const Pose2D curPose, const Pose2D prevPose, Pose2D& Motion){
  // 並進
  double dx = curPose.tx - prevPose.tx;
  double dy = curPose.ty - prevPose.ty;
  Motion.tx = prevPose.Rmat[0][0]*dx + prevPose.Rmat[1][0]*dy;
  Motion.ty = prevPose.Rmat[0][1]*dx + prevPose.Rmat[1][1]*dy;
  // 回転
  Motion.th = MyUtil::sub_angle(curPose.th, prevPose.th);
  Motion.calRmat();
}

// prevPoseからcurPoseまでの相対位置(移動距離)を求める -> Motion
// Motionはワールド座標系
void Pose2D::calGlobalMotion(const Pose2D curPose, const Pose2D prevPose, Pose2D& Motion){
  // 並進
  Motion.tx = curPose.tx - prevPose.tx;
  Motion.ty = curPose.ty - prevPose.ty;
  // 回転
  Motion.th = MyUtil::sub_angle(curPose.th, prevPose.th);
  Motion.calRmat();
}

// lastPose(t-1の位置)にMotionだけ足して(移動させて)、そこを予測位置とする -> predPose
void Pose2D::calPredPose(const Pose2D Motion, const Pose2D lastPose, Pose2D &predPose){
  // 並進
  double tx = Motion.tx;
  double ty = Motion.ty;
  predPose.tx =  lastPose.Rmat[0][0]*tx + lastPose.Rmat[0][1]*ty + lastPose.tx;
  predPose.ty =  lastPose.Rmat[1][0]*tx + lastPose.Rmat[1][1]*ty + lastPose.ty;
  // 回転
  predPose.th = MyUtil::add_angle(lastPose.th, Motion.th);
  predPose.calRmat();
}

// 自分（Pose2D）の局所座標系での点pを、グローバル座標系に変換してpoに入れる
void Pose2D::globalPoint(const LPoint2D &pi, LPoint2D &po) const {
  po.x = Rmat[0][0]*pi.x + Rmat[0][1]*pi.y + tx;
  po.y = Rmat[1][0]*pi.x + Rmat[1][1]*pi.y + ty;
}

// グローバル座標系での点pを、自分（Pose2D）の局所座標系に変換
LPoint2D Pose2D::relativePoint(const LPoint2D &p) const {
  double dx = p.x - tx;
  double dy = p.y - ty;
  double x = dx*Rmat[0][0] + dy*Rmat[1][0];  // 回転の逆行列
  double y = dx*Rmat[0][1] + dy*Rmat[1][1];
  return LPoint2D(p.sid, x, y);
}

// 自分（Pose2D）の局所座標系での点pを、グローバル座標系に変換
LPoint2D Pose2D::globalPoint(const LPoint2D &p) const {
  double x = Rmat[0][0]*p.x + Rmat[0][1]*p.y + tx;
  double y = Rmat[1][0]*p.x + Rmat[1][1]*p.y + ty;
  return LPoint2D(p.sid, x, y);
}
