#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h> 
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <math.h>
#include <armadillo>


using namespace std;
using namespace arma;

class RangeDetect{

private:
  
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher range_pub_;
  ros::Publisher x_wall_pub_;
  ros::Publisher y_wall_pub_;
   
  // for /scan
  //double scan_north_, scan_east_, scan_west_, scan_south_;
  //vector<float> scan_data_;
  float scan_range_;
  float scan_ang_;

  // for /odom
  double ang_z_;
  double pos_x_, pos_y_;
  
  // for /x_wall and /y_wall
  //x-direction and y-direction wall matrix, 0 is open, 1 is wall
  imat x_wall; 
  imat y_wall; 

  // const
  double PI = 3.1415926;
  double max_dist_ = 3.5;
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
  void odomCallback(const nav_msgs::Odometry& odomMsg);
  void publish();
  string drawWall(int draw, int xy);

public:
  
  RangeDetect(ros::NodeHandle& nh);
  
};
