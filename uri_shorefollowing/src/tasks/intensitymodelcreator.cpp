#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_shorefollowing/tasks/intensitymodelcreator.hpp>

namespace uri_shorefollowing{

  
  
  

IntensityModelCreator::IntensityModelCreator():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_shorefollowing::IntensityModelCreator";
	
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





TaskOutput IntensityModelCreator::_run(){
	
// 	std::cout << "a" << std::endl;
	
	// compute elapsed time since beginning and delta_t since last successful call
	bool terminate = false;
// 	double elapsed = (ros::Time::now() - start_t).toSec();
// 	delta_t = elapsed - last_elapsed;	
	
	// check if new laser scan is available. If not, terminate the execution of this _run 
	if (!ls->new_laser_available()){
		return uri::Continue;
	}
	// try to retrieve the scan. If the laser scanner is busy and does not respond in 0.001 seconds, terminate the execution of this _run
	sensor_msgs::LaserScan scan;
	if (!ls->get(scan, 0.001)){
		return uri::Continue;
	}
	
	TwoByNMatrix int_model;
	if (!intensity_model->get(int_model, 0.001)){
		return uri::Continue;
	}
	
	int_model.update(scan);
	
	std::stringstream ss;
	int_model.print(ss);
	out_file << ros::Time::now().toSec()-init_time << ss.str();
	
	intensity_model->set(int_model, 0.001);
	
	// set terminate at true to communicate to the behavior controller to terminate the execution of the task.
	if (terminate){
		return uri::Terminate;
	}
	// but usually, terminate is false.
	return uri::Continue;
}

void IntensityModelCreator::_activate(){
	
// 		std::cout << "a1" << std::endl;

	// what do you need to do every time the task is activated?
// 	last_elapsed = 0.0;
// 	start_t = ros::Time::now();
	
// 	std::cout << "a2" << std::endl;
	
}

void IntensityModelCreator::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void IntensityModelCreator::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	
	std::string lint("uri_sensors::LaserScanner");
	ls = (uri_sensors::LaserScanner*)res.get_resource_ptr(lint);
	
	std::string iint("uri_base::SharedMemory<uri_base::TwoByNMatrix>");
	intensity_model = (uri_base::SharedMemory<uri_base::TwoByNMatrix>*)res.get_resource_ptr(iint, "intensity_model");
	TwoByNMatrix int_mod;
	int_mod.initialize(0.25, 60, 0.0);
	if (not intensity_model->set(int_mod, 0.1)){
		ROS_ERROR("Unable to initialize Intensity Model.");
	}
	
	std::string mint("uri_uav_resources::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(mint);
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


