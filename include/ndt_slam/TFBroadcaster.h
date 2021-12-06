#ifndef TF_BROADCASTER_H_
#define TF_BROADCASTER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"

class TFBroadcaster {
public:
  tf::StampedTransform tf_map2odom;
  tf::StampedTransform tf_odom2base_link;
  tf::TransformBroadcaster br;

  TFBroadcaster() {
    tf_map2odom.frame_id_ = std::string("map");
    tf_map2odom.child_frame_id_ = std::string("odom");

    tf_odom2base_link.frame_id_ = std::string("odom");
    tf_odom2base_link.child_frame_id_ = std::string("base_link");
    Pose2D ini_pose(0.0, 0.0, 0.0);
    publish_tf_map2odom(ini_pose);
    //publish_tf_odom2base_link(ini_pose);
  }
  ~TFBroadcaster() {
  }

  void publish_tf_map2odom(Pose2D &pose) {
    tf_map2odom.stamp_ = ros::Time::now();
    tf_map2odom.setOrigin(tf::Vector3(pose.tx, pose.ty, 0.0));
    tf_map2odom.setRotation(tf::Quaternion(0.0, 0.0, DEG2RAD(pose.th))); // yaw, pitch, roll
    br.sendTransform(tf_map2odom); // 登録
  }
/*
  void publish_tf_odom2base_link(Pose2D &pose) {
    tf_odom2base_link.stamp_ = ros::Time::now();
    tf_odom2base_link.setOrigin(tf::Vector3(pose.tx, pose.ty, 0.0));
    tf_odom2base_link.setRotation(tf::Quaternion(0.0, 0.0, DEG2RAD(pose.th))); // yaw, pitch, roll
    br.sendTransform(tf_odom2base_link); // 登録
  }
*/
};

#endif
