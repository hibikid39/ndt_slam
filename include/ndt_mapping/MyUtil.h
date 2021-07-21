#ifndef MY_UTIL_H_
#define MY_UTIL_H_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <boost/array.hpp>

#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#ifndef M_PI
#define M_PI 3.14159265358979             // 円周率
#endif

#ifndef NULL
#define NULL 0                     // 基本的には、C++11のnullptrを使う
#endif

#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度

class MyUtil
{
public:
  MyUtil(void) {
  }

  ~MyUtil(void) {
  }

  // QuaternionからRPYに変換
  static void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat) {
  	tf::Quaternion quat;
  	quaternionMsgToTF(geometry_quat, quat);
  	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
  }

  // RPYからQuaternionに変換
  static geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw){
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
  }

  static void convertCov(const Eigen::Matrix3d &cov, boost::array<double, 36> &cov36) {
    for(int i=0; i<36; i++) cov36[i] = 0;

    cov36[0] = cov(0, 0)*200;
    cov36[1] = cov(0, 1)*200;
    cov36[6] = cov(0, 1)*200;
    cov36[7] = cov(1, 1)*200;
    cov36[5] = cov(0, 2)*200;
    cov36[30] = cov(0, 2)*200;
    cov36[11] = cov(1, 2)*200;
    cov36[31] = cov(1, 2)*200;
    cov36[35] = cov(2, 2)*200;
  }

///////////

  static double add_angle(double a1, double a2);
  static double sub_angle(double a1, double a2);
  static void calEigen2D(double (*mat)[2], double *vals, double *vec1, double *vec2);
  static Eigen::Matrix3d svdInverse(const Eigen::Matrix3d &A);
  static void printMatrix(const Eigen::Matrix3d &mat);

};

#endif
