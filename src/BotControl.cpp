#include <BotControl/BotControl.hpp>

BotControl::BotControl(ros::NodeHandle& nh)
{
  if(!loadParam()){
    ROS_ERROR("Error in loading the parameters.");
    ros::requestShutdown();
  }
  nh_ = nh;
  target_sub_ = nh_.subscribe<geometry_msgs::Point>("/pathplan/target", 1, &BotControl::targetCallBack, this);
  curr_sub_ = nh_.subscribe<geometry_msgs::Point>("/pathplan/curr", 1, &BotControl::currCallBack, this);
  control_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  // Kp_a = 0.04;
  // Ki_a = 0.2;
  // Kd_a = 0;
  // Kp_x = 0.5;
  // Ki_x = 0.01;
  // Kd_x = 0.09;

  ROS_INFO("bot_control_node initialized successfully.");
}


void BotControl::targetCallBack(const geometry_msgs::PointConstPtr& target_msg)
{
  target_x_ = target_msg->x;
  target_y_ = target_msg->y;
  goal_reached_ = target_msg->z;
  
  controlPub();

}


void BotControl::currCallBack(const geometry_msgs::PointConstPtr& curr_msg)
{
  pos_x_ = curr_msg->x;
  pos_y_ = curr_msg->y;
  heading_ = curr_msg->z; // in degree, with repsect to x axis in world frame
}


void BotControl::controlPub()
{
  
  double trans_x = 0, trans_heading = 0; // no control cmd
  
  error_pos_prev_ = error_pos_;
  error_heading_prev_ = error_heading_;
  
  //double error_x = target_x_ + 0.5 - pos_x_; //toward the center of the cell
  //double error_y = target_y_ + 0.5 - pos_y_;
  error_pos_ = target_x_; //sqrt(error_x*error_x + error_y*error_y);
  
  
  // the angle is with respect to the NORTH direction
  //double target_heading = atan2(error_y, error_x)* 180/PI; //in degree, with repsect to x axis in world frame
  error_heading_ = target_y_;//(target_heading - heading_) *PI/180; //in rad
  
  if(error_heading_ < -PI){
    error_heading_ += 2*PI;
  }
  if(error_heading_ > PI){
    error_heading_ -= 2*PI;
  }
  
  //implement a PID control here
  double I_heading = dt*error_heading_;
  double I_pos = dt*error_pos_;
  
  double D_heading = error_heading_prev_ - error_heading_;
  double D_pos = error_pos_prev_ - error_pos_;
  
  // if(fabs(error_heading_) > 0.2){
    
  //   trans_x = 0;
  //   trans_heading = Kp_a * error_heading_ + Ki_a * I_heading;

  //   if(fabs(error_heading_) > 0.2){
  //     trans_heading *= 7;
  //   }
    
  // }else{
  //   if(fabs(error_pos_) > 0.2){
  //     trans_x = Kp_x * error_pos_ + Ki_x * I_pos;
  //     trans_x *= pow(2, trans_x);
  //   }else{
  //     trans_x = Kp_x * error_pos_ + Ki_x * I_pos;
  //   }
  //   trans_heading = Kp_a * error_heading_ + Ki_a * I_heading; 
  // }
  
  trans_x = Kp_x * error_pos_ + Ki_x * I_pos + Kd_x * D_pos;
  trans_heading = Kp_a * error_heading_ + Ki_a * I_heading + Kd_a * D_heading; 
  
  if(trans_x > max_vel) trans_x = max_vel;
  if(trans_x < -max_vel) trans_x = -max_vel;
  if(trans_heading > max_ang) trans_heading = max_ang;
  if(trans_heading < -max_ang) trans_heading = -max_ang;
  
  geometry_msgs::Twist cmd;
  cmd.linear.x = trans_x;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = trans_heading;
  
  control_pub_.publish(cmd);
  ROS_INFO("pos_x_: %f, pos_y_: %f, heading_: %f", pos_x_, pos_y_, heading_);
  //ROS_INFO("target_heading:%f, error_pos_:%f, error_heading_:%f", target_heading, error_pos_, error_heading_);
  ROS_INFO("error_x:%f, error_heading: %f",target_x_, target_y_);

  ROS_INFO("trans_x:%f, trans_ang: %f",trans_x, trans_heading);
}


void BotControl::spin()
{
  ros::Rate loop_rate(1.0/dt);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}


bool BotControl::loadParam(){

	if(!nh_.getParam("/Kp_a", Kp_a)){
		ROS_ERROR("Kp_a Load Error");
		return false;
	}
	if(!nh_.getParam("/Ki_a", Ki_a)){
		ROS_ERROR("Ki_a Load Error");
		return false;
	}
	if(!nh_.getParam("/Kd_a", Kd_a)){
		ROS_ERROR("Kd_a Load Error");
		return false;
	}
	if(!nh_.getParam("/Kp_x", Kp_x)){
		ROS_ERROR("Kp_x Load Error");
		return false;
	}
	if(!nh_.getParam("/Ki_x", Ki_x)){
		ROS_ERROR("Ki_x Load Error");
		return false;
	}
	if(!nh_.getParam("/Kd_x", Kd_x)){
		ROS_ERROR("Kd_x Load Error");
		return false;
	}

	if(!nh_.getParam("/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}
