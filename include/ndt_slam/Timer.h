#ifndef TIMER_H_
#define TIMER_H_

#include <ros/ros.h>
#include <chrono>

using namespace std::chrono;

class Timer{
private:
  system_clock::time_point start;
  system_clock::time_point end;
//  system_clock::duration sum;
public:

  void start_timer(){
    start = system_clock::now();
  }
  void end_timer(){
    end = system_clock::now();
  }
  void print_timer(){
    auto dur = end - start;        // 要した時間を計算
    auto msec = std::chrono::duration_cast<milliseconds>(dur).count();
    ROS_INFO_STREAM("time = " << msec << "[ms]");
  }
/*
  void lap_timer(){
    auto dur = end - start;
    sum += dur;
  }
  void print_sum(){
    auto msec = std::chrono::duration_cast<milliseconds>(sum).count();
    ROS_INFO_STREAM("sum = " << sum << "[ms]");
  }
*/
};

#endif
