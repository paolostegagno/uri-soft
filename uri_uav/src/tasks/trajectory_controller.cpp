#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/trajectory_controller.hpp>

namespace uri_uav{

  
  
  

TrajectoryControllerTask::TrajectoryControllerTask():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_uav::TrajectoryControllerTask";
	
	// The class Task contains a field _options which you can use freely.
	// You can add (and pass options through the configuration file) with the following lines.
	// [ format: _options.addTYPEOption(OPTION_NAME, DEFAULT_VALUE); ]
	//
	_options.addDoubleOption("ki_x",0);
	_options.addDoubleOption("ki_y",0);
	_options.addDoubleOption("ki_z",0);
	_options.addDoubleOption("kp_x",0);
	_options.addDoubleOption("kp_y",0);
	_options.addDoubleOption("kp_z",0);
	_options.addDoubleOption("kd_x",0);
	_options.addDoubleOption("kd_y",0);
	_options.addDoubleOption("kd_z",0);
	// _options.addDoubleOption("name_option_2",0.05);
	// _options.addBoolOption("name_option_3",false);
	// _options.addStringOption("name_option_2","default_value");
	//
	// Note that options are only of those types listed here.
	// The first parameter in the above lines is the name, while the second parameter is the default value.
	// If the option is specified in the config file, the value of the option is automatically updated.
	// If the option specfied is not in the config_file, the option will have the default value
	// note that the options are updated after the execution of this constructor, so any option in this
	// construction will have its default value.
}

TaskOutput TrajectoryControllerTask::_run(){
	
	// Check if operating mode of the UAV is correct (GUIDED)
	if (not uav->guided()){
		if (uav->setMode("guided")) _guided_mode_requested=true;
		else _guided_mode_requested=false;
	}
	
	// get trajectory from shared memory
	if (trajectory->get(traj,0.0005)){
		if (not trajectory->ever_set()){
			traj.pos = uav->position();
		}
	}
	else {
		ROS_INFO("WARNING: %s unable to retrieve %s", _name.c_str(), traj.name().c_str());
// 		return Continue;
	}
// 	return Continue;
	
	Eigen::Vector3d pos = uav->position();
	double yaw = uav->yaw();
	
	double vx = _x_controller.run(pos(0), traj.pos(0), traj.vel(0), traj.vel(0));
	double vy = _y_controller.run(pos(1), traj.pos(1), traj.vel(1), traj.vel(1));
	double vz = _z_controller.run(pos(2), traj.pos(2), traj.vel(2), traj.vel(2));
		savefile  << " " << vx << " " << vy << " " << vz
							<< " " << pos(0) << " " << pos(1) << " " << pos(2)
							<< " " << traj.pos(0) << " " << traj.pos(1) << " " << traj.pos(2) 
							<< " " << yaw << " " << traj.yaw << " " << traj.yawrate << " " << (ros::Time::now().toSec()-_init_time.toSec()) << std::endl;
	
							
	
	uav->commandVelocity(vx, vy, vz);
	
	return Continue;
	
	
}

void TrajectoryControllerTask::_activate(){
	_x_controller.ki = 0.001;
	_x_controller.kp = 0.5;
	_x_controller.kd = 0.5;
	_y_controller.ki = 0.001;
	_y_controller.kp = 0.5;
	_y_controller.kd = 0.0;
	_z_controller.ki = 0.001;
	_z_controller.kp = 0.5;
	_z_controller.kd = 0.5;
	
	
	savefile.open("/home/paolos/savefile.txt", std::fstream::out | std::fstream::app);
	// what do you need to do every time the task is activated?
	
	_init_time = ros::Time::now();
}

void TrajectoryControllerTask::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void TrajectoryControllerTask::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	std::string iint("uri_uav_resources::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(iint);
	
	iint=std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);
}

};


