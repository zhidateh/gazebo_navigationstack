#include <BotControl/BotControl.hpp>

BotControl::BotControl(ros::NodeHandle& nh)
{
  if(!loadParam()){
    ROS_ERROR("Error in loading the parameters.");
    ros::requestShutdown();
  }
  nh_ = nh;
  target_sub_ = nh_.subscribe<geometry_msgs::Point>("/pathplan/control", 1, &BotControl::targetCallBack, this);
  control_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
  ROS_INFO("bot_control_node initialized successfully.");
}


void BotControl::targetCallBack(const geometry_msgs::PointConstPtr& target_msg)
{
  target_x_ = target_msg->x;
  target_y_ = target_msg->y;
  goal_reached_ = target_msg->z;
  
  controlPub();

}

void BotControl::controlPub()
{
  
  double trans_x = 0, trans_heading = 0; // no control cmd
  
  error_pos_prev_ = error_pos_;
  error_heading_prev_ = error_heading_;
  
  error_pos_ = target_x_; 
  error_heading_ = target_y_; 
  
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
