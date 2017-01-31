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
	_time_start = ros::Time::now();
}

TaskOutput Hover::_run(){
	
	uri_base::Trajectory traj;
	traj.pos = _goal_pos;
	traj.vel << 0,0,0;
	traj.acc << 0,0,0;
	traj.yaw = _goal_yaw;
	traj.yawrate = 0;
	
	trajectory->set(traj,0.01);
	
	if (_options["Countdown"]->getDoubleValue()>0.0){
		if ((ros::Time::now()-_time_start).toSec()>_options["Countdown"]->getDoubleValue()){
			return Terminate;
		}
	}
	
	return Continue;
	
}


void Hover::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);
	
	iint = std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);
	
}


};


