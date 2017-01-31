#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_shorefollowing/tasks/fixedspeeddesiredheadingcontroller.hpp>

namespace uri_shorefollowing{

  
  
  

FixedSpeedDesiredHeadingController::FixedSpeedDesiredHeadingController():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_shorefollowing::FixedSpeedDesiredHeadingController";
	
	// The class Task contains a field _options which you can use freely.
	// You can add (and pass options through the configuration file) with the following lines.
	// [ format: _options.addTYPEOption(OPTION_NAME, DEFAULT_VALUE); ]
	//
	// _options.addIntOption("name_option_1",15);
	// _options.addDoubleOption("name_option_2",0.05);
	// _options.addBoolOption("name_option_3",false);
	// _options.addStringOption("name_option_2","default_value");
	_options.addDoubleOption("speed",2.0);
	_options.addDoubleOption("max_acc",0.5);
	//
	// Note that options are only of those types listed here.
	// The first parameter in the above lines is the name, while the second parameter is the default value.
	// If the option is specified in the config file, the value of the option is automatically updated.
	// If the option specfied is not in the config_file, the option will have the default value
	// note that the options are updated after the execution of this constructor, so any option in this
	// construction will have its default value.
}

TaskOutput FixedSpeedDesiredHeadingController::_run(){
	
	
// 			std::cout << "FixedSpeedDesiredHeadingController::_run() a" << std::endl;

	// do your cool code here!
	
	bool terminate = false;
	double elapsed = (ros::Time::now() - start_t).toSec();
	delta_t = elapsed - last_elapsed;
	
	// if there is no heading input, then stay still by selecting the current position stored in the activation function.
	if (!desired_heading->ever_set()){
		traj->set(trajectory, 0.001);
	}
	// if here, then the heading is meaningful
	Heading heading_d;
	if (!desired_heading->get(heading_d, 0.001)){
		
	}
	
	double current_yaw;
	Eigen::Quaterniond ori = uav->orientation();
	uri_base::quaternion_to_yaw(ori, current_yaw);
	
	if (elapsed<max_speed_time){
		linear_acceleration = _options["max_acc"]->getDoubleValue();
		linear_speed = linear_acceleration*elapsed;
	}
	else {
		linear_acceleration = 0;
		linear_speed = _options["speed"]->getDoubleValue();
	}
	
	trajectory.acc(0) = linear_acceleration*cos(current_yaw);
	trajectory.acc(1) = linear_acceleration*sin(current_yaw);
	
	trajectory.vel(0) = linear_speed*cos(current_yaw);
	trajectory.vel(1) = linear_speed*sin(current_yaw);
	
	trajectory.pos = trajectory.pos + trajectory.vel*delta_t;
	
	trajectory.yaw = heading_d.heading;
	
	traj->set(trajectory, 0.005);
	
	
	last_elapsed = elapsed;
	
// 				std::cout << "FixedSpeedDesiredHeadingController::_run() b" << std::endl;

	
	
	if (terminate){
		return uri::Terminate;
	}
	return uri::Continue;
	// how to get the value of an option - use the following syntax:
	// _options["OPTION_NAME"]->getTYPEValue()
	//
	// example:
	// _options["name_option_2"]->getDoubleValue();
	//
}

void FixedSpeedDesiredHeadingController::_activate(){
	// what do you need to do every time the task is activated?
	
// 				std::cout << "FixedSpeedDesiredHeadingController::_activate() a" << std::endl;

	
	max_speed_time = _options["speed"]->getDoubleValue()/_options["max_acc"]->getDoubleValue();
	
	trajectory.vel(0) = 0.0;
	trajectory.vel(1) = 0.0;
	trajectory.vel(2) = 0.0;
	trajectory.acc(0) = 0.0;
	trajectory.acc(1) = 0.0;
	trajectory.acc(2) = 0.0;
	trajectory.pos = uav->position();
	Eigen::Quaterniond ori = uav->orientation();
	uri_base::quaternion_to_yaw(ori, trajectory.yaw);
	
	last_elapsed = 0.0;
	start_t = ros::Time::now();
// 					std::cout << "FixedSpeedDesiredHeadingController::_activate() b" << std::endl;

}

void FixedSpeedDesiredHeadingController::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void FixedSpeedDesiredHeadingController::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	std::string iint("uri_uav::IrisInterface");
	uav = (uri_uav::IrisInterface*)res.get_resource_ptr(iint);
	
	std::string lint("uri_base::SharedMemory<uri_base::Heading>");
	desired_heading = (uri_base::SharedMemory<uri_base::Heading>*)res.get_resource_ptr(lint);
	
	std::string mint("uri_base::SharedMemory<uri_base::Trajectory>");
	traj = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(mint);
	
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


