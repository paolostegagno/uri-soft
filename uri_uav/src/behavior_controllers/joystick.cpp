#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_uav/behavior_controllers/joystick.hpp>

namespace uri_uav{

  
  
  

Joystick::Joystick():BehaviorController()/*:_name(nm)*/{
	_name = "uri_uav::Joystick";
}




TaskOutput Joystick::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("ExampleBehavior1");
		return Continue;
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("ExampleBehavior1")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("ExampleBehavior2");
			return Continue;
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("ExampleBehavior2")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("ExampleBehavior1");
			return Continue;
		}
	}
	
}


void Joystick::get_mandatory_resources(ResourceVector &res){
	

}


};


