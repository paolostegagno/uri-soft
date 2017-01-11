#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_bridge/behavior_controllers/example_behavior_controller.hpp>

namespace uri_bridge{

  
  
  

ExampleBehaviorController::ExampleBehaviorController():BehaviorController()/*:_name(nm)*/{
	_name = "uri_bridge::ExampleBehaviorController";
}




TaskOutput ExampleBehaviorController::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("ExampleBehavior1");
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("ExampleBehavior1")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("ExampleBehavior2");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("ExampleBehavior2")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("ExampleBehavior1");
		}
	}
	
}


void ExampleBehaviorController::get_mandatory_resources(ResourceVector &res){
	

}


};


