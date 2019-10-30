#include <ros/ros.h>
#include <iostream>
#include "RangeDetect/RangeDetect.hpp"

int main(int argc, char** argv){
  
  //init a ros node
  ros::init(argc, argv, "range_detect_node");
  ros::NodeHandle nh;
  
  RangeDetect RD(nh);
  ros::spin();
  
  return 0;
}