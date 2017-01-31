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
	
// 	speed = 0.0;
}

TaskOutput ShoreFollowing::_run(){
	
	// do your cool code here!
	
	bool terminate = false;
	double elapsed = (ros::Time::now() - start_t).toSec();
	delta_t = elapsed - last_elapsed;	
	
	if (!ls->new_laser_available()){
		return uri::Continue;
	}
	sensor_msgs::LaserScan scan;
	if (!ls->get(scan, 0.001)){
		return uri::Continue;
	}
	
// 	std::cout << scan.header << std::endl;
	
	double current_yaw;
	Eigen::Quaterniond current_ori = uav->orientation();
	uri_base::quaternion_to_yaw(current_ori, current_yaw);
			last_current_yaw = current_yaw;

	
	// if a new pose estmate is available => integrateusing the new yaw as base
	if (current_yaw != last_current_yaw){
		heading_d.heading = current_yaw + 0.1*delta_t;
// 		std::cout << "a " << current_yaw << " " << last_current_yaw << " " <<  delta_t << " " << heading_d.heading << std::endl;
		last_current_yaw = current_yaw;
	}
	// if a new pose estimate is not available, then keep integrating what using the result of the previous step as base 
	else {
		last_current_yaw = current_yaw;
		heading_d.heading = heading_d.heading + 0.1*delta_t;
		if (heading_d.heading > M_PI) heading_d.heading = heading_d.heading - M_PI;
		if (heading_d.heading < -M_PI) heading_d.heading = heading_d.heading + M_PI;

// 		std::cout << "b " << current_yaw << " " << last_current_yaw << " " <<  delta_t << " " << heading_d.heading << std::endl;
	}
	desired_heading->set(heading_d, 0.001);
	
	
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	
	last_elapsed = elapsed;
	
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

void ShoreFollowing::_activate(){
	
// 		std::cout << "ShoreFollowing::_activate() a" << std::endl;

	// what do you need to do every time the task is activated?
	last_elapsed = 0.0;
	start_t = ros::Time::now();
	
	last_current_yaw = 0.0;
// 		std::cout << "ShoreFollowing::_activate() b" << std::endl;
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


