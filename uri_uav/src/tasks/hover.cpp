#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/hover.hpp>

namespace uri_uav{

  
  
  

Hover::Hover():Task()/*:_name(nm)*/{
	_name = "uri_uav::Hover";
	
	_first_run = true;
	
}




TaskOutput Hover::_run(){
	
// 	if (_first_run){
// 		uav->setMode("guided");
// 		_first_run = false;
// 		_pos_s = uav->position();
// 		_yaw_s = uav->yaw();
// 		_time_s = ros::Time::now();
// 		std::cout << _pos_s << std::endl;
// 	}
// 	
// 	double deltat = (ros::Time::now() - _time_s).toSec();
// 	
// 	_acc[0] = 0.0;
// 	_acc[1] = 0.0;
// 	_acc[2] = 0.0;
// 	
// 	_vel[0] = 0.3;
// 	_vel[1] = 0.2;
// 	_vel[2] = 0.0;
// 	
// 	_yawrate = 0.05;
// 	
// 	_yaw = _yaw_s + _yawrate * deltat;
// 	
// 	_pos = _pos_s + deltat*_vel;
// 	
// 	
// // 	uav->commandTrajectory(_pos, _vel, _acc, _yaw, _yawrate);
// // 	std::cout <<  deltat << " " << _pos.transpose() << " " << _yaw << std::endl;
// // 	uav->commandVelocity(0.5, 0.5, 0.0);
// 	
// 	Eigen::Quaterniond qu;
// 	qu.x() = 0.0;
// 	qu.y() = 0.0;
// 	qu.z() = 0.2;
// 	qu.w() = 0.95;
// 	std::cout <<  deltat << " " << qu.x() << " " << qu.z() << std::endl;
// 	
// // 	uav->commandAttitude(qu);
// 	
// 	if (deltat>5.0){
// 		return Terminate;
// 	}
	
	uri_base::Trajectory traj;
	traj.pos << 10,10,3;
	
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


