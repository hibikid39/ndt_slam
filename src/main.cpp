#include <ros/ros.h>

#include "ndt_mapping/SlamLauncher.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "ndt_mapping");

  SlamLauncher sl;
  sl.init();
  sl.loop_wait();

  ROS_INFO("End.");

  return 0;
}
