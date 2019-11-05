#include <ros/ros.h>
#include "PathPlan/PathPlan.hpp"
#include <math.h>
#include "PreDefine.hpp"

/*
 * Constructor
 */

PathPlan::PathPlan(ros::NodeHandle& nh)
{
  nh_ = nh;
  range_sub_ = nh_.subscribe("/range_pub", 1, &PathPlan::rangeCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &PathPlan::odomCallback, this);
  x_wall_sub_  = nh_.subscribe("/x_wall_pub", 1, &PathPlan::xwallCallback, this);
  y_wall_sub_  = nh_.subscribe("/y_wall_pub", 1, &PathPlan::ywallCallback, this);

  target_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/target", 1);
  curr_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/curr", 1);
  

  goal_reached_ = GOAL_NOT_REACH;
  
  x_wall.zeros(GRID_SIZE+1, GRID_SIZE); 
  y_wall.zeros(GRID_SIZE, GRID_SIZE+1);


  ROS_INFO("PathPlan node initialized successfully!");
}

void PathPlan::spin()
{
  ros::Rate loop_rate(1.0/dt);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/*
 * Callback Function
 */

void PathPlan::odomCallback(const nav_msgs::OdometryConstPtr& odomMsg)
{
  
  //use the first pose as the original point, with front pointing to y axis
  pos_y_ = odomMsg->pose.pose.position.y;
  pos_x_ = odomMsg->pose.pose.position.x;
  
  //ROS_INFO("Received /odom position (%f, %f).", pos_x_, pos_y_);
  
  float q_x = odomMsg->pose.pose.orientation.x;
  float q_y = odomMsg->pose.pose.orientation.y;
  float q_z = odomMsg->pose.pose.orientation.z;
  float q_w = odomMsg->pose.pose.orientation.w;
  ang_z_ = atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_z*q_z + q_y*q_y)); 

  //compute A star path and local path
  AStarPlanner();


  //check if A*star path has more than 1 coordinate
  if( rx.size()>1 && !goal_reached_){
    reverse(rx.begin(), rx.end());
    reverse(ry.begin(), ry.end());
    splinePath = CubicSpline::calcSplinePath(rx,ry,0.2);
    calcTargetIndex();
    tgt_dist = sqrt(pow(splinePath.rx.at(tgt_idx) - pos_x_,2) + pow(splinePath.ry.at(tgt_idx) - pos_y_,2));
    tgt_steer = normalizeAngle(splinePath.ryaw.at(tgt_idx) - ang_z_);

    ROS_INFO("--------A* Global Path--------");
    for(int i =0; i < rx.size() ;++i){
      ROS_INFO("X: %.2f, Y: %.2f", rx.at(i), ry.at(i));
    }
    ROS_INFO("--------Stanley Control---------");
    int number = 3;
    if( number > splinePath.rx.size()) number = splinePath.rx.size();
    for(int i =0; i < number ;++i){
      ROS_INFO("X: %.2f, Y: %.2f", splinePath.rx.at(i), splinePath.ry.at(i));
    }
    ROS_INFO("------------------------------");
    ROS_INFO("Target |X:%.2f,Y:%.2f,HDG:%.2f", splinePath.rx.at(tgt_idx), splinePath.ry.at(tgt_idx),splinePath.ryaw.at(tgt_idx));
    ROS_INFO("Robot  |X:%.2f,Y:%.2f,HDG:%.2f", pos_x_, pos_y_, ang_z_);
    ROS_INFO("Error  |front: %.2f, turn:%.2f", tgt_dist, tgt_steer);
  }
  else {
    ROS_INFO("---------Last approach----------");

    tgt_dist = sqrt ( pow(pos_x_ - (GOAL_X+map_shift),2)+ pow(pos_y_ - (GOAL_Y+map_shift),2));
    tgt_steer = normalizeAngle(atan2( (GOAL_Y+map_shift) - pos_y_ ,  (GOAL_X+map_shift) - pos_x_) - ang_z_  );
    
    ROS_INFO("------------------------------");
    ROS_INFO("Target |X:%.2f,Y:%.2f,HDG:- ", GOAL_X+map_shift, GOAL_Y+map_shift);
    ROS_INFO("Robot  |X:%.2f,Y:%.2f,HDG:%.2f", pos_x_, pos_y_, ang_z_);
    ROS_INFO("Error  |front: %.2f, turn:%.2f", tgt_dist, tgt_steer);

    //goal reached, set tgt dist and steer = 0
    if( fabs(pos_x_ - (GOAL_X+map_shift)) < goal_dist 
        && fabs(pos_y_ - (GOAL_Y+map_shift)) < goal_dist ){
      goal_reached_ = GOAL_REACH;
      tgt_dist = 0.0;
      tgt_steer = 0.0;
      ROS_INFO("****************************");
      ROS_INFO("Goal Reach!, X:%.2f, Y:%.2f", pos_x_, pos_y_);
      ROS_INFO("****************************");

    }
  }

  if( nearWall() && rB_phase == 0) rB_phase = 1; 
  if(rB_phase != 0 ) recoveryBehaviour();




  geometry_msgs::Point target_point;
  geometry_msgs::Point curr_position;
  
  target_point.x = tgt_dist;//target_x_;
  target_point.y = tgt_steer;//target_y_;
  target_point.z = goal_reached_;
  
  curr_position.x = pos_x_;
  curr_position.y = pos_y_;
  curr_position.z = ang_z_*180/PI; //in degree
  


  target_pub_.publish<geometry_msgs::Point>(target_point);
  curr_pub_.publish<geometry_msgs::Point>(curr_position);
  
}


void PathPlan::rangeCallback(const std_msgs::Float32& rangeMsg)
{
  min_ob_dist = rangeMsg.data;

}

void PathPlan::xwallCallback(const std_msgs::Int16MultiArray& xwallMsg)
{
  int data_size = xwallMsg.data.size();
  for(int i =0; i < data_size;i++){
    int j = i/(GRID_SIZE+1);
    x_wall( i%(GRID_SIZE+1),j ) = xwallMsg.data[i];
  }
}

void PathPlan::ywallCallback(const std_msgs::Int16MultiArray& ywallMsg)
{
  int data_size = ywallMsg.data.size();
  for(int i =0; i < data_size;i++){
    int j = i/(GRID_SIZE);
    y_wall( i%(GRID_SIZE),j ) = ywallMsg.data[i];
  }
}

void PathPlan::AStarPlanner()
{
  open_set.clear();
  close_set.clear();
  rx.clear();
  ry.clear();

  double position_x = pos_x_ - map_shift;
  double position_y = pos_y_ - map_shift;

  nstart = {  calcXYIndex(position_x,MIN_X), 
              calcXYIndex(position_y,MIN_Y),
              0.0,
              -1};

  ngoal = { calcXYIndex(GOAL_X ,MIN_X), 
            calcXYIndex(GOAL_Y ,MIN_Y),
            0.0,
            -1};

  open_set[calcGridIndex(nstart)] = nstart;

  while (true){
    if (open_set.size() == 0){
      int goal_id = calcMinCostIndex(close_set);
      break;
    }

    int c_id = calcMinCostIndex(open_set);
    Node current = open_set[c_id];

    if (current.x == ngoal.x && current.y == ngoal.y){
      //ROS_INFO("You have found your path");
      ngoal.pind = current.pind;
      ngoal.cost = current.cost;
      break;
    }

    open_set.erase(c_id);

    close_set[c_id] = current;

    for(int i =0; i < 4; i++){
      Node node = { current.x+ (int)motion[i][0],
                    current.y+ (int)motion[i][1],
                    current.cost+ motion[i][2],
                    c_id};

      int n_id = calcGridIndex(node);



      if(!checkCollision(current,node)) continue;

      if (close_set.count(n_id)) continue;

      if (!open_set.count(n_id)){
        open_set[n_id] = node;
      }
      else{
        if (open_set[n_id].cost > node.cost){
          open_set[n_id] = node;
        }
      }
    }
  }

  calcFinalPath();
}

int PathPlan::calcXYIndex(double position, double min_pos)
{
  return (int)(round(position-min_pos)/RESOLUTION);

}

int PathPlan::calcGridIndex(Node _node)
{
  return (int)((_node.y - MIN_Y) * GRID_SIZE + (_node.x-MIN_X));
}

float PathPlan::calcGridPosition(int index, double min_pos)
{
  return index * RESOLUTION + min_pos;
}


int PathPlan::calcMinCostIndex(map<int, Node> _set)
{
  map<int, Node> set = _set;
  map<int, Node>::iterator it;
  float min_cost = 9999.0;
  int min_idx = 9999;
  for ( it = set.begin(); it != set.end(); it++ ){
      float cost = it->second.cost + calcHeuristic(ngoal,it->second);
      if (cost < min_cost){
        min_cost = cost;
        min_idx = it->first;
      }
  }
  return min_idx;
}

float PathPlan::calcHeuristic(Node n1, Node n2)
{
  float w = 1.0; //weight of heuristic
  return w * sqrt(pow((n1.x-n2.x),2) + pow((n1.y-n2.y),2));

}

bool PathPlan::checkCollision(Node current_node, Node next_node)
{
  double px = calcGridPosition(next_node.x,MIN_X);
  double py = calcGridPosition(next_node.y,MIN_Y);
  if ( px >= GRID_SIZE || px < MIN_X ) return 0;
  if ( py >= GRID_SIZE || py < MIN_Y ) return 0;
  
  int dx = next_node.x - current_node.x;
  int dy = next_node.y - current_node.y;


  bool collision = false;
  if(dx > 0 && dy == 0) collision = x_wall(next_node.x,current_node.y);
  if(dx < 0 && dy == 0) collision = x_wall(current_node.x,current_node.y);
  if(dx == 0 && dy > 0) collision = y_wall(current_node.x,next_node.y);
  if(dx == 0 && dy < 0) collision = y_wall(current_node.x,current_node.y);


  if (collision){
    return 0;
  } 
  else return 1;

}

void PathPlan::calcFinalPath()
{
  rx.push_back(calcGridPosition(ngoal.x,MIN_X) + map_shift);
  ry.push_back(calcGridPosition(ngoal.y,MIN_Y) + map_shift);
  
  int pind = ngoal.pind;
  while (pind != -1){
    Node n = close_set[pind];
    rx.push_back(calcGridPosition(n.x,MIN_X)+map_shift);
    ry.push_back(calcGridPosition(n.y,MIN_Y)+map_shift);
    pind = n.pind;
  }
}


void PathPlan::calcTargetIndex()
{
  //search nearest point to spline path
  double min_dist = 9999;
  for(int i=0; i < splinePath.rx.size();++i){
    double d = sqrt( pow(pos_x_ + la_dist*cos(ang_z_) - splinePath.rx.at(i),2) +
                      pow(pos_y_ + la_dist*sin(ang_z_) - splinePath.ry.at(i),2));
    if (d < min_dist) {
      min_dist = d;
      tgt_idx = i;
    }
  }
}


double PathPlan::normalizeAngle(double angle){
	if (angle > PI){
		angle -= 2*PI;
		return normalizeAngle(angle);
	}
	else if(angle < -PI){
		angle += 2*PI;
		return normalizeAngle(angle);
	}
  return angle;
}

bool PathPlan::nearWall()
{

  if(min_ob_dist < collision_dist){
    return 1;
  }
  else return 0;
}

void PathPlan::recoveryBehaviour()
{

  ROS_INFO("!!! RECOVERY BEHAVIOUR IS TRIGGERED !!!");
  double min_dist = 9999;
  int nearest_idx = -1;
  //look for nearest nodes.
  for(int i =0; i < splinePath.rx.size();++i){
   double d = sqrt( pow(pos_x_  - splinePath.rx.at(i),2) +
                      pow(pos_y_  - splinePath.ry.at(i),2));
    if (d < min_dist) {
      min_dist = d;
      nearest_idx = i;
    }
  }


  switch(rB_phase){
    case 1:
      ROS_INFO("\t Phase 1");
  
      //adjust heading to face the nearest node
      // tgt_dist = 0.0;
      // if(min_ob_dist < 0.23) tgt_dist=-0.05;
      // tgt_steer = normalizeAngle(atan2( ry.at(nearest_idx) - pos_y_, rx.at(nearest_idx) -  pos_x_) - ang_z_);
      



      //move to the nearest node
      tgt_dist =  -sqrt( pow(pos_x_  - rx.at(nearest_idx),2) +
                      pow(pos_y_  - ry.at(nearest_idx),2));;
      tgt_steer = -normalizeAngle(atan2( ry.at(nearest_idx) - pos_y_, rx.at(nearest_idx) -  pos_x_) - ang_z_);

      if(abs(tgt_dist) > 0.4) rB_phase = 0;
      


      //if heading is the same, change to phase 2
      // if(abs(tgt_steer) < 0.05){
      //   rB_phase =2;
      // }
      break;

    case 2:
      
      ROS_INFO("\t Phase 2");

      //move to the nearest node
      tgt_dist =  sqrt( pow(pos_x_  - rx.at(nearest_idx),2) +
                      pow(pos_y_  - ry.at(nearest_idx),2));;
      tgt_steer = 0;

      if(tgt_dist > la_dist) tgt_dist = la_dist;

      //if the distance to nearest index is minimum, change to phase 3
      if (tgt_dist < 0.2) rB_phase = 0;
      break;

    default:
      return;
  }

  ROS_INFO("Target coordinate: X:%.2f, Y:%.2f", rx.at(nearest_idx),ry.at(nearest_idx));
  ROS_INFO("intend heading: %.2f", tgt_steer);
  ROS_INFO("error heading: %.2f", abs(tgt_steer));
  ROS_INFO("intend distance: %.2f", tgt_dist);

}

