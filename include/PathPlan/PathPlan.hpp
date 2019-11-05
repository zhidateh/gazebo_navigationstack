#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> //input from range_finder
#include <nav_msgs/Odometry.h> //input from odometry
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Point.h>  
#include <armadillo>
#include <math.h>
#include <stdio.h>

#include "CubicSpline.hpp"

struct Node{
  int x;
  int y;
  float cost;
  int pind;
};


using namespace std;
using namespace arma;

class PathPlan{
private:
  
  ros::NodeHandle nh_;
  ros::Subscriber range_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber x_wall_sub_;
  ros::Subscriber y_wall_sub_;
  
  ros::Publisher control_pub_;
  ros::Publisher target_pub_;
  ros::Publisher curr_pub_;
  
  /// Occupancy Grid/////---------------------------------------------------------
  //x-direction and y-direction wall matrix, 0 is open, 1 is wall
  imat x_wall; 
  imat y_wall; 
  float map_shift = 0.5;
  void xwallCallback(const std_msgs::Int16MultiArray& xwallMsg);
  void ywallCallback(const std_msgs::Int16MultiArray& ywallMsg);



  /// A Star calculation/////---------------------------------------------------------
  void AStarPlanner();
  float motion[4][3] = {{1,0,1},
                        {0,1,1},
                        {-1,0,1},
                        {0,-1,1}};

  map<int, Node> open_set, close_set;

  Node nstart,ngoal;
  int calcXYIndex(double position, double min_pos);
  int calcGridIndex(Node _node);
  float calcGridPosition(int index, double min_pos);
  int calcMinCostIndex(map<int, Node> _set);
  float calcHeuristic(Node n1, Node n2);
  bool checkCollision(Node current_node, Node next_node);
  void calcFinalPath();
  void calcTargetIndex();

  /// Variables/////------------------------------------------------------------------
  //global path (A star nodes)
  vector<double> rx,ry;

  //cubic spline path (Cubic spline nodes)
  CubicSpline::SplinePath splinePath;

  //look for target position
  double tgt_dist = 9999;
  double tgt_steer = 9999;
  double target_x, target_y, target_hdg; 
  int tgt_idx = -1;
  double la_dist = 0.25;      //look ahead distance
  double goal_dist = 0.25;    // goal radius distance
  int goal_reached_;        

  //for recovery behavior
  float collision_dist = 0.141; //robot radius
  bool nearWall();
  void recoveryBehaviour();
  int rB_phase = 0;
  bool rB_trigger = false;

  // callback functions
  void rangeCallback(const std_msgs::Float32MultiArray& rangeMsg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odomMsg);

  // for /odom
  double pos_x_, pos_y_, ang_z_; //current robot position and orientation in euler angle
  // for /range_pub
  float min_ob_dist=9999;
  float min_ob_ang=9999;
  
  //math function
  double normalizeAngle(double angle);

public:
  
  PathPlan(ros::NodeHandle& nh);
  void spin();

};
