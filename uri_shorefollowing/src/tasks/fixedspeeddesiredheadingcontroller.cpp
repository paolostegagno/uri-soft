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
	//
	// Note that options are only of those types listed here.
	// The first parameter in the above lines is the name, while the second parameter is the default value.
	// If the option is specified in the config file, the value of the option is automatically updated.
	// If the option specfied is not in the config_file, the option will have the default value
	// note that the options are updated after the execution of this constructor, so any option in this
	// construction will have its default value.
}

TaskOutput FixedSpeedDesiredHeadingController::_run(){
	// do your cool code here!
	
	bool terminate = false;
	char* chars;
	
	int numchar = keyboard->get(chars);
	
	if (numchar>0){
		std::cout << " read " << numchar << " chars:";
		for (int i =0; i<numchar; i++){
			std::cout << " " << chars[i];
			if (chars[i] == 'q'){
				terminate = true;
			}
		}
		std::cout << std::endl;
		
		delete chars;
	}
	
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
	
	std::string iint("uri_base::Keyboard");
	keyboard = (Keyboard*)res.get_resource_ptr(iint);
	
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


