#ifndef LPOINT2D_H_
#define LPOINT2D_H_

#include "MyUtil.h"

struct Vector2D {
  double x,y;
};


////////////////////////

enum ptype{UNKNOWN=0, LINE=1, CORNER=2, ISOLATE=3};    // 点のタイプ：未知、直線、コーナ、孤立

struct LPoint2D {
  int sid;                 // フレーム番号（スキャン番号）
  double x;                // 位置x
  double y;                // 位置y
  double nx;               // 法線ベクトル
  double ny;               // 法線ベクトル
  double atd;              // 累積走行距離(accumulated travel distance)
  ptype type;              // 点のタイプ

  LPoint2D() : sid(-1), x(0), y(0) {
    init();
  }

  LPoint2D(int id, double _x, double _y): x(_x), y(_y) {
    init();
    sid = id;
  }

  LPoint2D(double _x, double _y): x(_x), y(_y) {
    init();
  }

//////////

  void init() {
    sid = -1;
    atd = 0;
    type = UNKNOWN;
    nx = 0;
    ny = 0;
  }

  void setData(int _sid, double _x, double _y) {
    init();
    sid = _sid;
    x = _x;
    y = _y;
  }

  void set_RangeAngle2XY(double range, double angle) {
    double a = DEG2RAD(angle);
    x = range*cos(a);
    y = range*sin(a);
  }

  void setType(ptype t) {
    type = t;
  }

  void setNormal(double x, double y) {
    nx = x;
    ny = y;
  }

};

#endif
