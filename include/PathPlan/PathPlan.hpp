#include <ros/ros.h>
#include <std_msgs/Float32.h> //input from range_finder
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
  
  ros::Publisher target_pub_;
  ros::Publisher curr_pub_;
  
  /////////////////////////////
  //x-direction and y-direction wall matrix, 0 is open, 1 is wall
  imat x_wall; 
  imat y_wall; 
  float map_shift = 0.5;
  void xwallCallback(const std_msgs::Int16MultiArray& xwallMsg);
  void ywallCallback(const std_msgs::Int16MultiArray& ywallMsg);

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

  //global path
  vector<double> rx,ry;

  CubicSpline::SplinePath splinePath;
  void calcTargetIndex();
  double normalizeAngle(double angle);
  double tgt_dist = 9999;
  double tgt_steer = 9999;
  int tgt_idx = -1;
  double la_dist = 0.25;
  double goal_dist = 0.25;
  /////////////////////////////////////////////////

  float min_ob_dist;
  float collision_dist = 0.141;
  bool nearWall();
  void recoveryBehaviour();
  int rB_phase = 0;
  bool rB_trigger = false;

  // callback functions
  void rangeCallback(const std_msgs::Float32& rangeMsg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odomMsg);



  cube wall_map; // It's a 3D tensor (can create as row, col, slices)
  
  // for /odom
  double pos_x_, pos_y_, ang_z_; //current robot position and orientation in euler angle
  // for /range_pub
  double dist_north_, dist_east_, dist_south_, dist_west_;
  
  int goal_reached_;

  // update the map
  void initializeWall();
  void setWall(int x, int y, int direction);
  void removeWall(int x, int y, int direction);
  bool hasWall(int x, int y, int direction);
  

  //Path Planning Algorithm
  Mat<int> path_map_;
  vec neighbor_value_ = vec(4, fill::zeros);
  int target_x_, target_y_, target_x_prev_, target_y_prev_;
  bool path_map_initialized_ = false;
  void initializePathMap(); // set all the cell with 1000
  void setNextDestCell(); // based on the current position, find the next heading cell
  int getPathMapValue(int x, int y);
  
public:
  
  PathPlan(ros::NodeHandle& nh);
  void spin();
  
  void checkWall();
  void PathPlanAlgorithm();  
};
