#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/hover.hpp>

#include <uri_base/angle_conversion.hpp>


namespace uri_uav{

  
  
  

Hover::Hover():Task()/*:_name(nm)*/{
	_name = "uri_uav::Hover";
	
	_first_run = true;
	
	_options.addStringOption("GoalSource", "options"); // possible values are trajectory, pose, options
	_options.addDoubleOption("X", 0.0);
	_options.addDoubleOption("Y", 0.0);
	_options.addDoubleOption("Z", 3.0);
	_options.addDoubleOption("Yaw", 0.0);
	_options.addDoubleOption("YawRate", 0.0);
	_options.addDoubleOption("Countdown", -1.0);
	
}


void Hover::_activate(){
// 	std::cout << " activate!!!! " << std::endl; 
	
	if (_options["GoalSource"]->getStringValue().compare("options") == 0) { // get goal from options
		_goal_pos(0) = _options["X"]->getDoubleValue();
		_goal_pos(1) = _options["Y"]->getDoubleValue();
		_goal_pos(2) = _options["Z"]->getDoubleValue();
		_goal_yaw = _options["Yaw"]->getDoubleValue();
	}
	else if (_options["GoalSource"]->getStringValue().compare("trajectory") == 0) {  // get goal from current desired trajectory
		uri_base::Trajectory traj;
		if (trajectory->get(traj, 0.001) && trajectory->ever_set()){
			_goal_pos = traj.pos;
			_goal_yaw = traj.yaw;
		}
		else {
			_goal_pos = uav->position();
			Eigen::Quaterniond ori = uav->orientation();
			uri_base::quaternion_to_yaw(ori, _goal_yaw);
		}
	}
	else {  // get goal from current pose
		_goal_pos = uav->position();
		Eigen::Quaterniond ori = uav->orientation();
		uri_base::quaternion_to_yaw(ori, _goal_yaw);
	}
// 	std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA " << _goal_yaw << std::endl;
	_time_start = ros::Time::now();
	_prev_time = _time_start;
}


void Hover::_reset(){
	_time_start = ros::Time::now();
	_prev_time = _time_start;
}


TaskOutput Hover::_run(){
	
	uri_base::Trajectory traj;
	traj.pos = _goal_pos;
	traj.vel << 0,0,0;
	traj.acc << 0,0,0;
	traj.yawrate = _options["YawRate"]->getDoubleValue();
	
	double delta_t ;
	if (traj.yawrate != 0.0){
		ros::Time _time_now = ros::Time::now();
		delta_t = (_time_now - _prev_time).toSec();
		_goal_yaw = _goal_yaw + traj.yawrate*delta_t;
		while  (_goal_yaw >  M_PI) _goal_yaw -= 2*M_PI;
		while  (_goal_yaw < -M_PI) _goal_yaw += 2*M_PI;
		traj.yawrate = 0.0;
		_prev_time = _time_now;
	}
	
	traj.yaw = _goal_yaw;
// 	std::cout << "a "<< _goal_yaw << " " << delta_t << std::endl;
	
	trajectory->set(traj,0.01);
	
	if (_options["Countdown"]->getDoubleValue()>0.0){
		if ((ros::Time::now()-_time_start).toSec()>_options["Countdown"]->getDoubleValue()){
			return Terminate;
		}
	}
	
	return Continue;
	
}


void Hover::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav_resources::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(iint);
	
	iint = std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);
	
	
}


};


