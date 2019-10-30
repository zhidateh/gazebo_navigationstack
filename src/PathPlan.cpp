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
  
  // target_x_ = 0;
  // target_y_ = 0;
  // target_x_prev_ = 0;
  // target_y_prev_ = 0;
  

  goal_reached_ = GOAL_NOT_REACH;
  
  x_wall.zeros(GRID_SIZE+1, GRID_SIZE); 
  y_wall.zeros(GRID_SIZE, GRID_SIZE+1);

  //initializeWall();
  //initializePathMap();
  
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
  //because in the world frame, there is a 90 degree transformation
  //mind that ang_z_ is with respect to the x axis of the world frame
  
  // check the wall distribution and update the path map every time the robot localize itself
  //checkWall();  
  
  //PathPlanAlgorithm();

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
    tgt_dist = sqrt ( pow(pos_x_ - (GOAL_X+map_shift),2)+ pow(pos_y_ - (GOAL_Y+map_shift),2));
    tgt_steer = normalizeAngle(atan2( (GOAL_Y+map_shift) - pos_y_ ,  (GOAL_X+map_shift) - pos_x_));
    
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

  if(nearWall()) rB_phase = 1;
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
  // dist_north_ = rangeMsg.data[0];
  // dist_east_ = rangeMsg.data[1];
  // dist_south_ = rangeMsg.data[2];
  // dist_west_ = rangeMsg.data[3];
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

  // if (current_node.x == 0){
  //   ROS_INFO("COllision :%d", y_wall(0,7));
  //   ROS_INFO("dy :%d", dy);
  //   ROS_INFO("dx :%d", dx);

  //   ROS_INFO("Current X:%d, Next X:%d", current_node.x, next_node.x);
  //   ROS_INFO("Current Y:%d, Next Y:%d", current_node.y, next_node.y);
  //   ROS_INFO("COllision :%d", collision);

  // }

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
  if(min_ob_dist < collision_dist) return 1;
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
      tgt_dist = 0;
      tgt_steer = normalizeAngle(atan2( ry.at(nearest_idx) - pos_y_, rx.at(nearest_idx) -  pos_x_) - ang_z_);
      
    

      //if heading is the same, change to phase 2
      if(abs(tgt_steer) < 0.05){
        
        std::cout << " am i here" << std::endl;
        
        rB_phase =0;
      }
      break;

    case 2:
      
      ROS_INFO("\t Phase 2");

      //move to the nearest node
      tgt_dist = sqrt( pow(pos_x_  - rx.at(nearest_idx),2) +
                      pow(pos_y_  - ry.at(nearest_idx),2));;
      tgt_steer = 0;


      //if the distance to nearest index is minimum, change to phase 3
      if (tgt_dist < 0.2) rB_phase = 0;
      break;

    case 3:
      break;
      ROS_INFO("\t Phase 3");

      //align heading to the correspoding node's heading
      tgt_dist = 0;
      tgt_steer = normalizeAngle(splinePath.ryaw.at(tgt_idx) - ang_z_);
      
      //if heading is the same, recovery is complete
      if(abs(normalizeAngle(tgt_steer-ang_z_)) < 0.005) rB_phase =0;
      break;

    default:
      return;
  }

  ROS_INFO("Target coordinate: X:%.2f, Y:%.2f", rx.at(nearest_idx),ry.at(nearest_idx));
  ROS_INFO("intend heading: %.2f", tgt_steer);
  ROS_INFO("current heading: %.2f", abs(tgt_steer));
  ROS_INFO("intend distance: %.2f", tgt_dist);

}


// /*
//  * Wall Map related
//  */
// void PathPlan::checkWall()
// {
//   //in world frame
//   int pos_x_int = (int)floor(pos_x_);
//   int pos_y_int = (int)floor(pos_y_);
  
//   if(dist_north_ < WALL_DETECT_DIST){
//     setWall(pos_x_int, pos_y_int, NORTH);
//   } 

//   if(dist_north_ > OPEN_DETECT_DIST){
//     removeWall(pos_x_int, pos_y_int, NORTH);
//   }
  
//   if(dist_east_ < WALL_DETECT_DIST){
//     setWall(pos_x_int, pos_y_int, EAST);
//   } 

//   if(dist_east_ > OPEN_DETECT_DIST){
//     removeWall(pos_x_int, pos_y_int, EAST);
//   }
  
//   if(dist_south_ < WALL_DETECT_DIST){
//     setWall(pos_x_int, pos_y_int, SOUTH);
//   } 
//   if(dist_south_ > OPEN_DETECT_DIST){
//     removeWall(pos_x_int, pos_y_int, SOUTH);
//   }
  
//   if(dist_west_ < WALL_DETECT_DIST){
//     setWall(pos_x_int, pos_y_int, WEST);
//   } 
//   if(dist_west_ > OPEN_DETECT_DIST){
//     removeWall(pos_x_int, pos_y_int, WEST);
//   }
  
// }


// void PathPlan::setWall(int x, int y, int direction)
// {
//   if(x<0 || y <0 || direction <0){
//     ROS_ERROR("setWall function input with illegal: x:%d, y:%d, direction:%d", x, y, direction);
//     return;
//   }
//   wall_map(x, y, direction) = WALL;
//   ROS_INFO("Wall in %d", direction);
  
//   // fill wall to adjacent cells
//   if(direction == NORTH){
//     if(y+1 < GRID_SIZE) wall_map(x, y+1, SOUTH) = WALL;
//     return;
//   }
//   if(direction == SOUTH){
//     if(y-1 >= 0) wall_map(x, y-1, NORTH) = WALL;
//     return;
//   }
//   if(direction == WEST){
//     if(x-1 >= 0) wall_map(x-1, y, EAST) = WALL;
//     return;
//   }
//   if(direction == EAST){
//     if(x+1 < GRID_SIZE) wall_map(x+1, y, WEST) = WALL;
//     return;
//   }
  
// }


// bool PathPlan::hasWall(int x, int y, int direction)
// {
//   if(x<0 || y<0 || direction<0){
//     ROS_ERROR("hasWall get an illegal input: x:%d, y:%d, direction:%d", x, y, direction);
//     return true; //return with positive wall
//   }
//   if(wall_map(x, y, direction) == WALL) {
//     return true;
//   }
//   else return false;
// }

// void PathPlan::removeWall(int x, int y, int direction)
// {
//   if(x<0 || y<0 || direction<0){
//     ROS_ERROR("removeWall get an illegal input: x:%d, y:%d, direction:%d", x, y, direction);
//     return;
//   }
//   wall_map(x, y, direction) = OPEN;
// }


// void PathPlan::initializeWall()
// {
//   //mind that the map we draw has a different orientation with the cell matrix 
//   //each cell has four direction, with 0 on one direction means open on that direction
//   wall_map = cube(GRID_SIZE, GRID_SIZE, 4, fill::zeros); 
  
//   for(int row = 0; row < GRID_SIZE; row++){
//     for(int col = 0; col < GRID_SIZE; col++){
//       if(row == 0){
// 	wall_map(row, col, WEST) = WALL;
//       }
//       if(row == GRID_SIZE-1){
// 	wall_map(row, col, EAST) = WALL;
//       }
//       if(col == 0){
// 	wall_map(row, col, SOUTH) = WALL;
//       }
//       if(col == GRID_SIZE-1){
// 	wall_map(row, col, NORTH) = WALL;
//       }
//     }
//   }
// }


// /*
//  * path map related
//  */
// void PathPlan::initializePathMap()
// {
//   path_map_ = Mat<int>(GRID_SIZE, GRID_SIZE); //set all the path value as inf
//   path_map_.fill(1000);
//   path_map_(GOAL_X, GOAL_Y) = 0;
//   path_map_initialized_ = true;
// }


// void PathPlan::PathPlanAlgorithm()
// {
//   // make sure the goal cell has been put in the path_map
//   if(!path_map_initialized_) initializePathMap();
  
//   mat visited_map(GRID_SIZE, GRID_SIZE, fill::zeros);
//   mat is_queued(GRID_SIZE, GRID_SIZE, fill::zeros);
  
//   std::vector<pair<int, int>> reached_queue;
//   reached_queue.push_back(pair<int, int>(GOAL_X, GOAL_Y));
  
//   int nVisited = 0;
//   int nCells = GRID_SIZE * GRID_SIZE;
//   int mdist_from_goal_node = 0;
//   pair<int, int> node;
  
//   int x_queue, y_queue; //tmp coord in queue
  
//   while(nVisited < nCells){
//     int min_dist = 1000;
//     int queue_length = reached_queue.size();
    
//     for(int i = 0; i<queue_length; i++){
//       pair<int, int> tmp = reached_queue[i];
//       if(visited_map.at(tmp.first, tmp.second) == 0){
//       	if(path_map_(tmp.first, tmp.second) < min_dist){
//       		min_dist = path_map_.at(tmp.first, tmp.second);
//       		node = tmp;
//       		}
//       	}
//     }
    
//     visited_map(node.first, node.second) = 1; //this node has been reached
//     nVisited++;
    
//     // extend to adjacent cells to this node
//     for(int direction = NORTH; direction <= WEST; direction++){

//     	if(!hasWall(node.first, node.second, direction)){

//     		// there is no wall on that direction, and the adjacent node hasn't been put in the queue_length
//     		if(direction == NORTH && node.second+1 < GRID_SIZE){
//     			x_queue = node.first; y_queue = node.second+1;
//     		}
//     		if(direction == EAST && node.first+1 < GRID_SIZE){
//     			x_queue = node.first+1; y_queue = node.second;
//     		}
//     		if(direction == SOUTH && node.second-1 >= 0){
//     			x_queue = node.first; y_queue = node.second-1;
//     		}
//     		if(direction == WEST && node.first-1 >= 0){
//     			x_queue = node.first-1; y_queue = node.second;
//     		}

//     		if(!is_queued(x_queue, y_queue)){
//     			// put the adjacent cell in the queue
//     			reached_queue.push_back(pair<int, int>(x_queue, y_queue));
//     			is_queued(x_queue, y_queue) = 1;
//     		}

//     		// update the path_map
//     		path_map_(x_queue, y_queue) = min(path_map_(x_queue, y_queue), min_dist+1);
//     	}
	
//     }
    
//   }
  
// }


// void PathPlan::setNextDestCell()
// {
//   //locate the current cell
//   int pos_x_int = (int)floor(pos_x_);
//   int pos_y_int = (int)floor(pos_y_);
  
//   if(pos_x_int == GOAL_X && pos_y_int == GOAL_Y){
//     goal_reached_ = GOAL_REACH;
//     ROS_INFO("Goal Reach!, X:%d, Y:%d", GOAL_X, GOAL_Y);
//     return;
//   }
  
//   ROS_INFO("pos_x_int:%d, pos_y_int:%d", pos_x_int, pos_y_int);
  
//   neighbor_value_(NORTH) = getPathMapValue(pos_x_int, pos_y_int+1);
//   neighbor_value_(EAST) = getPathMapValue(pos_x_int+1, pos_y_int);
//   neighbor_value_(SOUTH) = getPathMapValue(pos_x_int, pos_y_int-1);
//   neighbor_value_(WEST) = getPathMapValue(pos_x_int-1, pos_y_int);
  
//   int min_heading = UNDEFINED;
//   int min_path_value = 1000;
  
//   for(int direction = NORTH; direction <= WEST; direction++){
//     if(!hasWall(pos_x_int, pos_y_int, direction)){
//       if(neighbor_value_(direction) <= min_path_value){

//       	min_path_value = neighbor_value_(direction);
//       	min_heading = direction;

//       }
//     }
//   }
  
//   if(min_heading < 0){ //haven't find correct path, then go back
//     target_x_ = target_x_prev_;
//     target_y_ = target_y_prev_;
//     } else{ //update the next target destination
//     target_x_prev_ = target_x_;
//     target_y_prev_ = target_y_;
    
//     if(min_heading == NORTH){target_x_ = pos_x_int; target_y_ = pos_y_int+1;}
//     if(min_heading == EAST){target_x_ = pos_x_int+1; target_y_ = pos_y_int;}
//     if(min_heading == SOUTH){target_x_ = pos_x_int; target_y_ = pos_y_int-1;}
//     if(min_heading == WEST){target_x_ = pos_x_int-1; target_y_ = pos_y_int;}
    
//   }
  
//   ROS_INFO("Next destination: (%d, %d)", target_x_, target_y_);
  
// }

// int PathPlan::getPathMapValue(int x, int y)
// {
//   if(x<0 || x>=GRID_SIZE || y<0 || y>=GRID_SIZE){
//     ROS_ERROR("get_path_map_value get an illegal input x:%d, y%d", x, y);
//     return 1000;
//   }
//   if(!path_map_initialized_){
//     ROS_ERROR("path_map hasn't initialized.");
//     return 1000;
//   }
  
//   return path_map_.at(x, y);
  
// }
