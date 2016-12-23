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
	
	_options.addBoolOption("GetGoalPoseFromTrajectory", true);
	_options.addDoubleOption("X", 0.0);
	_options.addDoubleOption("Y", 0.0);
	_options.addDoubleOption("Z", 3.0);
	_options.addDoubleOption("Yaw", 0.0);
	
}


void Hover::_activate(){
	_goal_pos;
	uri_base::Trajectory traj;
	if (trajectory->get(traj, 0.001)){
		_goal_pos = traj.pos;
		_goal_yaw = traj.yaw;
	}
	else {
		_goal_pos = uav->position();
		Eigen::Quaterniond ori = uav->orientation();
		uri_base::quaternion_to_yaw(ori, _goal_yaw);
	}
}

TaskOutput Hover::_run(){
	
	uri_base::Trajectory traj;
	traj.pos = _goal_pos;
	traj.vel << 0,0,0;
	traj.acc << 0,0,0;
	traj.yaw = _goal_yaw;
	traj.yawrate = 0;
	
	trajectory->set(traj,0.01);
	
	return Continue;
	
}


void Hover::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);
	
	iint = std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);
	
}


};


