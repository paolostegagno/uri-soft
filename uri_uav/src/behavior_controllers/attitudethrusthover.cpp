#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_uav/behavior_controllers/attitudethrusthover.hpp>

namespace uri_uav{

  
  
  

AttitudeThrustHover::AttitudeThrustHover():BehaviorController()/*:_name(nm)*/{
	_name = "uri_uav::AttitudeThrustHover";
}




TaskOutput AttitudeThrustHover::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Takeoff");
		return Continue;
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Hover");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate())
			_next_active_behavior = behavior("Hover");
	}
}


void AttitudeThrustHover::get_mandatory_resources(ResourceVector &res){
	

}


};


