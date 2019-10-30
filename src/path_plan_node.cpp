#include <ros/ros.h>
#include "PathPlan/PathPlan.hpp"

int main(int argc, char** argv){
  
  ros::init(argc, argv, "path_plan_node");
  ros::NodeHandle nh;
  
  PathPlan PP(nh);
  
  PP.spin();
  
  return 0;
}