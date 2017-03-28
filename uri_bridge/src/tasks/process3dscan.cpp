#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_bridge/tasks/process3dscan.hpp>

namespace uri_bridge{

  
  
  

Process3DScan::Process3DScan():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_bridge::Process3DScan";
	
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

TaskOutput Process3DScan::_run(){
	// do your cool code here!
	bool terminate = false;
	
	uri_bridge::PointCloud pc;
	pcl->get(pc, 1.0);
	
	
	
	return uri::Terminate;
}

void Process3DScan::_activate(){
	// what do you need to do every time the task is activated?
}

void Process3DScan::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void Process3DScan::get_mandatory_resources(ResourceVector &res){
	
	std::string pint("uri_base::SharedMemory<uri_bridge::PointCloud>");
	pcl = (uri_base::SharedMemory<uri_bridge::PointCloud>*)res.get_resource_ptr(pint);
	
}

};


