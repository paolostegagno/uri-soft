

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/PoseStamped.h"

#include <sstream>
#include <iostream>
#include <string>

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"

#include "uri_uav/resources/iris_interface.hpp"

#include <bitset>




namespace uri_uav{

	
	

//#################################################################################################
//###  all ROS callback functions goes here   #####################################################
//#################################################################################################
	
void IrisInterface::_timer_interpolate_poses_CB(const ros::TimerEvent& event){
	double elapsed = (ros::Time::now() - start_t).toSec();
	delta_t = elapsed - last_elapsed;
	
	// integrate position
	_position[0] += delta_t*_velocity_lin(0);
	_position[1] += delta_t*_velocity_lin(1);
	_position[2] += delta_t*_velocity_lin(2);
	
	// integrate yaw
	double r, p, y;
	uri_base::quaternion_to_rpy(_orientation, r, p, y);
	y +=  delta_t*_velocity_ang(2);
	while (y >  M_PI) y -= 2*M_PI;
	while (y < -M_PI) y += 2*M_PI;
	_orientation = uri_base::rpy_to_quaternion(r, p, y);
	
// 	std::cout << "pos " << delta_t << " " << _position[0] << " " << _position[1] << " " << _position[2] << std::endl;
	
	// update last time we integrated the pose
	last_elapsed = elapsed;
// 	_local_position_pose_received = true;
	
}


void IrisInterface::_local_position_pose_CB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	_position(0)=msg->pose.position.x;
	_position(1)=msg->pose.position.y;
	_position(2)=msg->pose.position.z;
	_orientation.w() = msg->pose.orientation.w;
	_orientation.x() = msg->pose.orientation.x;
	_orientation.y() = msg->pose.orientation.y;
	_orientation.z() = msg->pose.orientation.z;
	_local_position_pose_received = true;
}

void IrisInterface::_local_position_velocity_CB(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	_velocity_lin(0)=msg->twist.linear.x;
	_velocity_lin(1)=msg->twist.linear.y;
	_velocity_lin(2)=msg->twist.linear.z;
	_velocity_ang(0)=msg->twist.angular.x;
	_velocity_ang(1)=msg->twist.angular.y;
	_velocity_ang(2)=msg->twist.angular.z;
}

void IrisInterface::_state_CB(const mavros_msgs::State::ConstPtr& msg)
{
	_armed = msg->armed;
	_connected = msg->connected;
	_guided = msg->guided;
	_mode = msg->mode;
}

void IrisInterface::_battery_CB(const mavros_msgs::BatteryStatus::ConstPtr& msg)
{
	double alpha = 0.8;
	_battery_voltage = alpha*_battery_voltage + (1-alpha)*msg->voltage;
	_battery_current = alpha*_battery_current + (1-alpha)*msg->current;
	_battery_remaining = alpha*_battery_remaining + (1-alpha)*msg->remaining;
}


//#################################################################################################
//###   all constructors goes here   ##############################################################
//#################################################################################################
IrisInterface::IrisInterface(ros::NodeHandle &_n):Resource(_n)
{
  // create all publishers here
	_pub_setpoint_velocity_cmd_vel = n->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity_ext/cmd_vel_ext", 1);
	
	_pub_setpoint_attitude_ext_cmd_vel = n->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude_ext/cmd_vel", 1);
	_pub_setpoint_attitude_ext_attitude = n->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude_ext/attitude", 1);
	_pub_setpoint_attitude_ext_att_throttle = n->advertise<std_msgs::Float64>("/mavros/setpoint_attitude_ext/att_throttle", 1);
	
	// here all publishers which writes on topics read by mavros
	_pub_setpoint_raw_local = n->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
	_pub_setpoint_raw_global = n->advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 1);
	_pub_setpoint_raw_attitude = n->advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
	_pub_setpoint_raw_ext_local_ext = n->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw_ext/local_ext", 1);
	_pub_setpoint_raw_ext_global_ext = n->advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw_ext/global_ext", 1);
	_pub_setpoint_raw_ext_attitude_ext = n->advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw_ext/attitude_ext", 1);
	
	

	// subscribe to all topics published by mavros
	_sub_local_position_pose = n->subscribe("/mavros/local_position/pose", 1, &IrisInterface::_local_position_pose_CB, this);
	_local_position_pose_received = false;
	_sub_local_position_velocity = n->subscribe("/mavros/local_position/velocity", 1, &IrisInterface::_local_position_velocity_CB, this);

	// find all services provided by mavros
	_srv_set_mode = n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	_srv_cmd_arming = n->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	_srv_cmd_takeoff = n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	_srv_cmd_land = n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	
	last_elapsed = 0.0;
	start_t = ros::Time::now();
	_timer_interpolate_poses = n->createTimer(ros::Duration(0.02), &IrisInterface::_timer_interpolate_poses_CB, this, false, false);
};




IrisInterface::IrisInterface()
{
	_name = "uri_uav::IrisInterface";
};



void IrisInterface::_init()
{
	// create all publishers here
	_pub_setpoint_velocity_cmd_vel = n->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity_ext/cmd_vel_ext", 1);
	
	_pub_setpoint_attitude_ext_cmd_vel = n->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude_ext/cmd_vel", 1);
	_pub_setpoint_attitude_ext_attitude = n->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude_ext/attitude", 1);
	_pub_setpoint_attitude_ext_att_throttle = n->advertise<std_msgs::Float64>("/mavros/setpoint_attitude_ext/att_throttle", 1);
	
	// here all publishers which writes on topics read by mavros
	_pub_setpoint_raw_local = n->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
	_pub_setpoint_raw_global = n->advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 1);
	_pub_setpoint_raw_attitude = n->advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
	_pub_setpoint_raw_ext_local_ext = n->advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw_ext/local_ext", 1);
	_pub_setpoint_raw_ext_global_ext = n->advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw_ext/global_ext", 1);
	_pub_setpoint_raw_ext_attitude_ext = n->advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw_ext/attitude_ext", 1);

	// subscribe to all topics published by mavros
	_sub_local_position_pose = n->subscribe("/mavros/local_position/pose", 1, &IrisInterface::_local_position_pose_CB, this);
	_local_position_pose_received = false;
	_sub_local_position_velocity = n->subscribe("/mavros/local_position/velocity", 1, &IrisInterface::_local_position_velocity_CB, this);
	
	_sub_state = n->subscribe("/mavros/state", 1, &IrisInterface::_state_CB, this);
	_connected = false;
	_armed = false;
	_guided = false;
	_mode = "";

	_sub_battery = n->subscribe("/mavros/battery", 1, &IrisInterface::_battery_CB, this);
	_battery_voltage = 0.0;
	_battery_current = 0.0;
	_battery_remaining = 0.0;

  
  // find all services provided by mavros
  _srv_set_mode = n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  _srv_set_stream_rate = n->serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
  _srv_cmd_arming = n->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  _srv_cmd_takeoff = n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

	// what do you need to do every time the task is activated?
	last_elapsed = 0.0;
	start_t = ros::Time::now();
	_timer_interpolate_poses = n->createTimer(ros::Duration(0.02), &IrisInterface::_timer_interpolate_poses_CB, this, false, false);
	
// 	this->setRate(6);
};


// void IrisInterface::initialize(ros::NodeHandle& _n)
// {
// 	n = &_n;
// 
//   // create all publishers here
//   _pub_setpoint_velocity_cmd_vel = n->advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity_ext/cmd_vel_ext", 1);
//   
//   // subscribe to all topics published by mavros
//   _sub_local_position_pose = n->subscribe("/mavros/local_position/pose", 1, &IrisInterface::_local_position_pose_CB, this);
//   _local_position_pose_received = false;
//   
//   // find all services provided by mavros
//   _srv_set_mode = n->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
//   _srv_cmd_arming = n->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//   _srv_cmd_takeoff = n->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
// 
// };



//#################################################################################################
//###   all public function for interfaceing with the main goes here   ############################
//#################################################################################################
bool IrisInterface::setMode(std::string mode)
{
  _msg_set_mode.request.custom_mode = mode;
  if (_srv_set_mode.call(_msg_set_mode))
  {
    ROS_INFO("called setMode service. Result: %d", _msg_set_mode.response.success);
    return _msg_set_mode.response.success;
  }
  else
  {
    ROS_ERROR("Failed to call service set_mode");
    return false;
  }
}

//#################################################################################################
bool IrisInterface::setRate(unsigned int rate)
{
	_msg_set_stream_rate.request.stream_id = (uint8_t)0;
	_msg_set_stream_rate.request.message_rate = rate;
	_msg_set_stream_rate.request.on_off = 1;
	if (_srv_set_stream_rate.call(_msg_set_stream_rate))
	{
		ROS_INFO("called set_stream_rate service.");
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service set_stream_rate");
		return false;
	}
}

//#################################################################################################
bool IrisInterface::setRateRawSensors(unsigned int rate)
{
	_msg_set_stream_rate.request.stream_id = (uint8_t)1;
	_msg_set_stream_rate.request.message_rate = rate;
	_msg_set_stream_rate.request.on_off = 1;
	if (_srv_set_stream_rate.call(_msg_set_stream_rate))
	{
		ROS_INFO("called set_stream_rate service.");
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service set_stream_rate");
		return false;
	}
}

//#################################################################################################
bool IrisInterface::setRatePosition(unsigned int rate)
{
	_msg_set_stream_rate.request.stream_id = (uint8_t)6;
	_msg_set_stream_rate.request.message_rate = rate;
	_msg_set_stream_rate.request.on_off = 6;
	if (_srv_set_stream_rate.call(_msg_set_stream_rate))
	{
		ROS_INFO("called set_stream_rate service.");
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service set_stream_rate");
		return false;
	}
}


//#################################################################################################
bool IrisInterface::setRateExtendedStatus(unsigned int rate)
{
	_msg_set_stream_rate.request.stream_id = (uint8_t)2;
	_msg_set_stream_rate.request.message_rate = rate;
	_msg_set_stream_rate.request.on_off = 1;
	if (_srv_set_stream_rate.call(_msg_set_stream_rate))
	{
		ROS_INFO("called set_stream_rate service.");
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service set_stream_rate");
		return false;
	}
}


//#################################################################################################
bool IrisInterface::armThrottle()
{
  _msg_cmd_arming.request.value = true;
  if (_srv_cmd_arming.call(_msg_cmd_arming))
  {
    ROS_INFO("called arming service. Result: %d %d", _msg_cmd_arming.response.success, _msg_cmd_arming.response.result);
    return _msg_cmd_arming.response.success;
  }
  else
  {
    ROS_ERROR("Failed to call service cmd_arming");
    return false;
  }
}

//#################################################################################################
bool IrisInterface::disarmThrottle()
{
  _msg_cmd_arming.request.value = false;
  if (_srv_cmd_arming.call(_msg_cmd_arming))
  {
    ROS_INFO("called disarming service. Result: %d %d", _msg_cmd_arming.response.success, _msg_cmd_arming.response.result);
    return _msg_cmd_arming.response.success;
  }
  else
  {
    ROS_ERROR("Failed to call service cmd_arming");
    return false;
  }
}


//#################################################################################################
bool IrisInterface::takeoff(double altitude)
{
  _msg_cmd_takeoff.request.altitude = altitude;
  if (_srv_cmd_takeoff.call(_msg_cmd_takeoff))
  {
    ROS_INFO("called service takeoff. Result: %d %d", _msg_cmd_takeoff.response.success, _msg_cmd_takeoff.response.result);
		return _msg_cmd_takeoff.response.success;
  }
  else
  {
    ROS_ERROR("Failed to call service cmd_takeoff");
    return false;
  }
}


//#################################################################################################
bool IrisInterface::land()
{
//   _msg_cmd_land.request.altitude;
  if (_srv_cmd_land.call(_msg_cmd_land))
  {
    ROS_INFO("called service land. Result: %d %d", _msg_cmd_land.response.success, _msg_cmd_land.response.result);
		return _msg_cmd_land.response.success;
  }
  else
  {
    ROS_ERROR("Failed to call service cmd_land");
    return false;
  }
}



//#################################################################################################
void IrisInterface::commandVelocity(double x, double y, double z)
{
  geometry_msgs::TwistStamped _msg_setpoint_velocity_cmd_vel;

  _msg_setpoint_velocity_cmd_vel.twist.linear.x = x;
  _msg_setpoint_velocity_cmd_vel.twist.linear.y = y;
  _msg_setpoint_velocity_cmd_vel.twist.linear.z = z;
  _msg_setpoint_velocity_cmd_vel.twist.angular.z = -1;
  
  _pub_setpoint_velocity_cmd_vel.publish<geometry_msgs::TwistStamped>(_msg_setpoint_velocity_cmd_vel);
}

void IrisInterface::commandYawrate(double yr)
{
	mavros_msgs::AttitudeTarget _msg_setpoint_raw_attitude;
	
	char a = -58;    
	std::bitset<8> x(a);
	std::cout << x;

	short c = -315;
	std::bitset<16> y(c);
	std::cout << y;
	
	uint8_t mask = (1 << 7) | (1 << 6) | (1 << 1) | (1 << 0);
	
	std::cout << "!!! " << mask << std::endl;
	
	_msg_setpoint_raw_attitude.body_rate.x = 0;
	_msg_setpoint_raw_attitude.body_rate.y = 0;
	_msg_setpoint_raw_attitude.body_rate.z = yr;
	_msg_setpoint_raw_attitude.orientation.x = 0;
	_msg_setpoint_raw_attitude.orientation.y = 0;
	_msg_setpoint_raw_attitude.orientation.z = 0;
	_msg_setpoint_raw_attitude.orientation.w = 0;
	_msg_setpoint_raw_attitude.type_mask = (uint8_t)64;
//   _msg_setpoint_velocity_cmd_vel.twist.linear.y = y;
//   _msg_setpoint_velocity_cmd_vel.twist.linear.z = z;
//   _msg_setpoint_velocity_cmd_vel.twist.angular.z = -1;
  
	_pub_setpoint_raw_attitude.publish<mavros_msgs::AttitudeTarget>(_msg_setpoint_raw_attitude);
}



void IrisInterface::commandTrajectory(Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a, double yaw, double yawrate)
{
	mavros_msgs::PositionTarget msg;

	msg.position.x = p[0];
	msg.position.y = p[1];
	msg.position.z = p[2];

	msg.velocity.x = v[0];
	msg.velocity.y = v[1];
	msg.velocity.z = v[2];

	msg.acceleration_or_force.x = a[0];
	msg.acceleration_or_force.y = a[1];
	msg.acceleration_or_force.z = a[2];

	msg.yaw = yaw;
	msg.yaw_rate = yawrate;
	
	msg.type_mask=0;
	msg.coordinate_frame=1;
	
  _pub_setpoint_raw_ext_local_ext.publish<mavros_msgs::PositionTarget>(msg);
}




}; // end namespace




