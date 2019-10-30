#include "RangeDetect/RangeDetect.hpp"
#include "PreDefine.hpp"

RangeDetect::RangeDetect(ros::NodeHandle& nh)
{
  nh_ = nh;
  scan_sub_ = nh_.subscribe("/scan", 1, &RangeDetect::scanCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &RangeDetect::odomCallback, this);
  range_pub_ = nh_.advertise<std_msgs::Float32>("/range_pub", 1);
  x_wall_pub_ = nh_.advertise<std_msgs::Int16MultiArray>("/x_wall_pub", 1);
  y_wall_pub_ = nh_.advertise<std_msgs::Int16MultiArray>("/y_wall_pub", 1);

  //x-direction and y-direction wall matrix, 0 is open, 1 is wall  
  x_wall.zeros(GRID_SIZE+1, GRID_SIZE); //= zeros<imat>(GRID_SIZE+1, GRID_SIZE+1);
  y_wall.zeros(GRID_SIZE, GRID_SIZE+1);

  ROS_INFO("range_detect_node initialized successfully");
}

/*
 * scanCallback Function
 */
void RangeDetect::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
  
  vector<float> scan_data_ = scanMsg->ranges;
  double scan_increment = scanMsg->angle_increment;
  
  int scan_size = scan_data_.size();
  
  /////////////////////LASER SLAM//////////////////////
  //transform scan data to robot frame
  for(int i = 0; i < scan_size; i += 1){
    if(isinf(scan_data_[i])) continue;

    //transform angle from [0,360] to [-pi,pi]

    double angle = (scan_increment * i);

    if(angle > PI) angle -= 2*PI;

    double x = scan_data_[i] * cos(angle);
    double y = scan_data_[i] * sin(angle);

    //rotate with robot heaading
    double x_inertia = x * cos(ang_z_) - y*sin(ang_z_);
    double y_inertia = x * sin(ang_z_) + y*cos(ang_z_);

    //translate with robot position
    x_inertia += pos_x_;
    y_inertia += pos_y_;
    
    if ( fmod(x_inertia,0.5) > 0.4 || fmod(x_inertia,0.5) < 0.1 ){
      int x_idx = static_cast<int> (round(x_inertia/0.5)*5)/5; 
      int y_idx = static_cast<int> (round(y_inertia/0.5)*5)/5; 
      if( x_idx%2 == 0 && y_idx %2){
        //x_wall x_idx:0,2,4,6,8,10,12 ...
        //x_wall y_idx:1,3,5,7,9, ...
        x_wall(x_idx/2, (y_idx-1)/2) = 1;
      }
      if (x_idx%2 && y_idx%2 == 0){
        //y_wall x_idx: 1,3,5,7...
        //y_wall y_idx:0, 2, 4, 6, ...
        y_wall((x_idx-1)/2, y_idx/2) = 1;
      }
    }

  }
  

/////////////////COLLISION CHECKING FOR RECOVERY BEHAVIOUR/////////////////
  scan_data_ = scanMsg->ranges;
	int arr_size = scan_data_.size();
	float smallest_dist = 100;

	for(int i = 0; i<arr_size; i++){
		if(scan_data_[i] < smallest_dist) {
			smallest_dist = scan_data_[i];
			scan_ang_ = scanMsg->angle_min + scanMsg->angle_increment*i;
		}
	}
	scan_range_ = smallest_dist;
  ////////////////////////////////////////////////////////////////////////

  publish();
  ROS_INFO("--------------------------------------");
  ROS_INFO("Nearest obstace distance: %.2f", scan_range_ );
  ROS_INFO("Occupancy grid");
  for(int i=9; i>=0; i--){

    if(i != 9){
      ROS_INFO("%s%s%s%s%s%s%s%s%s%s", 
              drawWall(y_wall(i,9),1).c_str(),
              drawWall(y_wall(i,8),1).c_str(),
              drawWall(y_wall(i,7),1).c_str(),
              drawWall(y_wall(i,6),1).c_str(),
              drawWall(y_wall(i,5),1).c_str(),
              drawWall(y_wall(i,4),1).c_str(),
              drawWall(y_wall(i,3),1).c_str(),
              drawWall(y_wall(i,2),1).c_str(),
              drawWall(y_wall(i,1),1).c_str(),
              drawWall(y_wall(i,0),1).c_str()
              );
    }
    ROS_INFO("%s%s%s%s%s%s%s%s%s", 
            drawWall(x_wall(i,8),0).c_str(),
            drawWall(x_wall(i,7),0).c_str(),
            drawWall(x_wall(i,6),0).c_str(),
            drawWall(x_wall(i,5),0).c_str(),
            drawWall(x_wall(i,4),0).c_str(),
            drawWall(x_wall(i,3),0).c_str(),
            drawWall(x_wall(i,2),0).c_str(),
            drawWall(x_wall(i,1),0).c_str(),
            drawWall(x_wall(i,0),0).c_str()
            );
    }


  /*
  
  scan_data_ = scanMsg->ranges;
  double scan_increment = scanMsg->angle_increment;
  
  int scan_size = scan_data_.size();
  int east_index = 0, north_index = scan_size/4, west_index = scan_size/2, south_index = scan_size*3/4;
  
  int increment = (int) (ang_z_ / scan_increment);
  
  east_index -= increment;
  north_index -= increment;
  west_index -= increment;
  south_index -= increment;
  
  while(east_index < 0) east_index += scan_size;
  while(north_index < 0) north_index += scan_size;
  while(west_index < 0) west_index += scan_size;
  while(south_index < 0) south_index += scan_size;
  
  while(east_index > scan_size) east_index -= scan_size;
  while(north_index > scan_size) north_index -= scan_size;
  while(west_index > scan_size) west_index -= scan_size;
  while(south_index > scan_size) south_index -= scan_size;
  
  scan_east_ = scan_data_[east_index];
  scan_north_ = scan_data_[north_index];
  scan_west_ = scan_data_[west_index];
  scan_south_ = scan_data_[south_index];
  
  if(isinf(scan_east_)) scan_east_ = max_dist_;
  if(isinf(scan_north_)) scan_north_ = max_dist_;
  if(isinf(scan_west_)) scan_west_ = max_dist_;
  if(isinf(scan_south_)) scan_south_ = max_dist_;
  
  ROS_INFO("E:%f, N:%f, W:%f, S:%f", scan_east_, scan_north_, scan_west_, scan_south_);
  
  publish();
  */
}

void RangeDetect::odomCallback(const nav_msgs::Odometry& odomMsg)
{
  double qx = odomMsg.pose.pose.orientation.x;
  double qy = odomMsg.pose.pose.orientation.y;
  double qz = odomMsg.pose.pose.orientation.z;
  double qw = odomMsg.pose.pose.orientation.w;
  
  ang_z_ = atan2(2*(qw*qz+qx*qy), 1-2*(qz*qz+qy*qy));
  
  pos_x_ = odomMsg.pose.pose.position.x;
  pos_y_ = odomMsg.pose.pose.position.y;  
}

void RangeDetect::publish()
{
  std_msgs::Int16MultiArray x_pub;
  std_msgs::Int16MultiArray y_pub;

  x_pub.data.clear();
  y_pub.data.clear();
  
  for(int j =0; j < GRID_SIZE; j++){
    for(int i =0; i < GRID_SIZE+1; i++){
      x_pub.data.push_back(x_wall(i,j));
    }
  }
  for(int j =0; j < GRID_SIZE+1; j++){
    for(int i =0; i < GRID_SIZE; i++){
      y_pub.data.push_back(y_wall(i,j));
    }
  }

  x_wall_pub_.publish<std_msgs::Int16MultiArray>(x_pub);
  y_wall_pub_.publish<std_msgs::Int16MultiArray>(y_pub);
  
  std_msgs::Float32 msg_pub;
  //msg_pub.data.clear();
  // msg_pub.data.push_back(scan_north_);
  // msg_pub.data.push_back(scan_east_);
  // msg_pub.data.push_back(scan_south_);
  // msg_pub.data.push_back(scan_west_);
  //msg_pub.data.push_back(scan_range_);

  msg_pub.data = scan_range_;
  range_pub_.publish(msg_pub);

}

string RangeDetect::drawWall(int draw, int xy){
  if(draw && !xy) return "_";
  else if(draw && xy) return "|";
  else return " ";
}