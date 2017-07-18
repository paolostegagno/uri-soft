#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/yawratecontroller.hpp>

namespace uri_uav{

  
  
  

YawrateController::YawrateController():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_uav::YawrateController";
	
	// The class Task contains a field _options which you can use freely.
	// You can add (and pass options through the configuration file) with the following lines.
	// [ format: _options.addTYPEOption(OPTION_NAME, DEFAULT_VALUE); ]
	//
	// _options.addIntOption("name_option_1",15);
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

TaskOutput YawrateController::_run(){
	// do your cool code here!

	if (not uav->guided()){
		if (uav->setMode("guided")) _guided_mode_requested=true;
		else _guided_mode_requested=false;
	}
	
	
	if (trajectory->get(traj,0.0005)){
		if (not trajectory->ever_set()){
			traj.yaw = uav->yaw();
		}
	}
	else {
		ROS_INFO("WARNING: %s unable to retrieve %s", _name.c_str(), traj.name().c_str());
	}
	
// 	uav->commandYawrate(traj.yawrate);
	uav->commandYawrate(0.5);
	
}

void YawrateController::_activate(){
	// what do you need to do every time the task is activated?
}

void YawrateController::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void YawrateController::get_mandatory_resources(ResourceVector &res){
	
	
	// to get the resources needed in this task, use the following method:
	std::string iint("uri_uav_resources::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(iint);
	
	iint=std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);
}

};


