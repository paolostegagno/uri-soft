#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_shorefollowing/tasks/shorefollowing.hpp>

namespace uri_shorefollowing{

  
  
  

ShoreFollowing::ShoreFollowing():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_shorefollowing::ShoreFollowing";
	
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
	
	// how to get the value of an option - use the following syntax:
	// _options["OPTION_NAME"]->getTYPEValue()
	//
	// example:
	// _options["name_option_2"]->getDoubleValue();
	//
}

TaskOutput ShoreFollowing::_run(){
	
	// compute elapsed time since beginning and delta_t since last successful call
	bool terminate = false;
	double elapsed = (ros::Time::now() - start_t).toSec();
	delta_t = elapsed - last_elapsed;	
	
	// check if new laser scan is available. If not, terminate the execution of this _run 
	if (!ls->new_laser_available()){
		return uri::Continue;
	}
	// try to retrieve the scan. If the laser scanner is busy and does not respond in 0.001 seconds, terminate the execution of this _run
	sensor_msgs::LaserScan scan;
	if (!ls->get(scan, 0.001)){
		return uri::Continue;
	}
	
	// if we made it this far, scan contains a new scan! we should decide the new heading based on it
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// the lines below simply make the UAV go in circle
	heading_d.heading = heading_d.heading + 0.1*delta_t;
	// normalize the heading_d between -M_PI and M_PI
	while (heading_d.heading > M_PI) heading_d.heading = heading_d.heading - 2*M_PI;
	while (heading_d.heading < -M_PI) heading_d.heading = heading_d.heading + 2*M_PI;
	
	// here we set the desired heading in the shared memory - the controller will use it
	desired_heading->set(heading_d, 0.001);
	
	// update last time  we computed the heading
	last_elapsed = elapsed;
	
	// set terminate at true to communicate to the behavior controller to terminate the execution of the task.
	if (terminate){
		return uri::Terminate;
	}
	// but usually, terminate is false.
	return uri::Continue;
}

void ShoreFollowing::_activate(){
	
	// what do you need to do every time the task is activated?
	last_elapsed = 0.0;
	start_t = ros::Time::now();
	
	// select first desired heading as the current yaw
	double current_yaw;
	Eigen::Quaterniond current_ori = uav->orientation();
	uri_base::quaternion_to_yaw(current_ori, current_yaw);
	heading_d.heading = current_yaw;
}

void ShoreFollowing::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void ShoreFollowing::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	
	std::string lint("uri_sensors::LaserScanner");
	ls = (uri_sensors::LaserScanner*)res.get_resource_ptr(lint);
	
	std::string iint("uri_base::SharedMemory<uri_base::Heading>");
	desired_heading = (uri_base::SharedMemory<uri_base::Heading>*)res.get_resource_ptr(iint);
	
	std::string mint("uri_uav::IrisInterface");
	uav = (uri_uav::IrisInterface*)res.get_resource_ptr(mint);
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


