#ifndef POSE2D_H_
#define POSE2D_H_

#include <ros/ros.h>

#include "MyUtil.h"
#include "LPoint2D.h"

/////////

struct Pose2D {
  double tx;                           // 並進x
  double ty;                           // 並進y
  double th;                           // 回転角(度)
  double Rmat[2][2];                   // 姿勢の回転行列

  Pose2D() : tx(0), ty(0), th(0) {
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++) {
        Rmat[i][j] = (i==j)? 1.0:0.0;
      }
    }
  }

  Pose2D(double tx, double ty, double th) {
    this->tx = tx;
    this->ty = ty;
    this->th = th;
    calRmat();
  }

  Pose2D(double mat[2][2], double tx, double ty, double th) {
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++) {
        Rmat[i][j] = mat[i][j];
      }
    }
    this->tx = tx;
    this->ty = ty;
    this->th = th;
  }

  void calRmat(){
    double a = DEG2RAD(th);
    Rmat[0][0] = Rmat[1][1] = std::cos(a);
    Rmat[1][0] = std::sin(a);
    Rmat[0][1] = -Rmat[1][0];
  }

  void setPose(double x, double y, double a) {
    tx = x;
    ty = y;
    th = a;
    calRmat();
  }

/////////////////

  static void calMotion(Pose2D curScan, Pose2D prevScan, Pose2D& Motion);
  static void calGlobalMotion(const Pose2D curPose, const Pose2D prevPose, Pose2D& Motion);
  static void calPredPose(Pose2D Motion, Pose2D lastPose, Pose2D& predPose);
  void globalPoint(const LPoint2D &pi, LPoint2D &po) const;
  LPoint2D relativePoint(const LPoint2D &p) const;
  LPoint2D globalPoint(const LPoint2D &p) const;

};

#endif
