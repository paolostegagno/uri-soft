#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
// #include "gazebo_msgs/GetModelState.h"
// #include "tf_conversions/tf_eigen.h"
// #include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <Eigen/Geometry>

#include <uri_base/angle_conversion.hpp>


// #include "telekyb_msgs/TKState.h"
// #include "telekyb_srvs/BinOccupancySrv.h"

// #define COEF 0.5
// #define YAWCOEF 1.0

// bool saveFirstState = true;
// gazebo_msgs::ModelState firstState;
// bool resetState = false;

gazebo_msgs::ModelState setModelState;
// gazebo_msgs::GetModelState getModelState;

ros::Publisher* gazeboPub;
// ros::ServiceClient* gazeboClient;

ros::Subscriber _sub_local_position_pose;
ros::Subscriber _sub_local_position_velocity;


double timerDuration;

Eigen::Vector3d _position;
Eigen::Quaterniond _orientation;

Eigen::Vector3d _velocity_lin;
Eigen::Vector3d _velocity_ang;


void _local_position_pose_CB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	_position(0)=msg->pose.position.x;
	_position(1)=msg->pose.position.y;
	_position(2)=msg->pose.position.z;
	_orientation.w() = msg->pose.orientation.w;
	_orientation.x() = msg->pose.orientation.x;
	_orientation.y() = msg->pose.orientation.y;
	_orientation.z() = msg->pose.orientation.z;
}


void _local_position_velocity_CB(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	_velocity_lin(0)=msg->twist.linear.x;
	_velocity_lin(1)=msg->twist.linear.y;
	_velocity_lin(2)=msg->twist.linear.z;
	_velocity_ang(0)=msg->twist.angular.x;
	_velocity_ang(1)=msg->twist.angular.y;
	_velocity_ang(2)=msg->twist.angular.z;
}



void stateTimerCB(const ros::TimerEvent&)
{
	_position[0] += timerDuration*_velocity_lin(0);
	_position[1] += timerDuration*_velocity_lin(1);
	_position[2] += timerDuration*_velocity_lin(2);
	
	double r, p, y;
	uri_base::quaternion_to_rpy(_orientation, r, p, y);
	y +=  timerDuration*_velocity_ang(2);
	while (y >  M_PI) y -= 2*M_PI;
	while (y < -M_PI) y += 2*M_PI;
	_orientation = uri_base::rpy_to_quaternion(r, p, y);
    
	setModelState.model_name = "iris";
	setModelState.pose.position.x = _position[0];// = getModelState.response.pose;
	setModelState.pose.position.y = _position[1];// = getModelState.response.pose;
	setModelState.pose.position.z = _position[2]+0.25;// = getModelState.response.pose;
	setModelState.pose.orientation.x = _orientation.x();// = getModelState.response.pose;
	setModelState.pose.orientation.y = _orientation.y();// = getModelState.response.pose;
	setModelState.pose.orientation.z = _orientation.z();// = getModelState.response.pose;
	setModelState.pose.orientation.w = _orientation.w();// = getModelState.response.pose;
	setModelState.twist.linear.x = _velocity_lin(0);
	setModelState.twist.linear.y = _velocity_lin(1);
	setModelState.twist.linear.z = _velocity_lin(2);
	setModelState.twist.angular.z = _velocity_ang(2);
    
	gazeboPub->publish(setModelState);
 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazeboInterface");
	ros::NodeHandle n;
  
	timerDuration = 0.01;
  
//   getModelState.request.model_name = "qc13";
//   getModelState.request.relative_entity_name = "world";
	
	
	_position << 0.0, 0.0, 0.0;
	
	_orientation.x() = 0.0;
	_orientation.y() = 0.0;
	_orientation.z() = 0.0;
	_orientation.w() = 1.0;
	
	_sub_local_position_pose = n.subscribe("/mavros/local_position/pose", 1, _local_position_pose_CB/*, this*/);
	_sub_local_position_velocity = n.subscribe("/mavros/local_position/velocity", 1, _local_position_velocity_CB/*, this*/);
	
	gazeboPub = new ros::Publisher(n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",10));
//   gazeboClient = new ros::ServiceClient(n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"));
	
	ros::Timer stateTimer = n.createTimer(ros::Duration(timerDuration), stateTimerCB); 
	
	ros::spin();

  return 0;
}
