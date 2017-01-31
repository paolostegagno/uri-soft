#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_shorefollowing/behavior_controllers/shorefollowingexperiment.hpp>

namespace uri_shorefollowing{

  
  
  

ShoreFollowingExperiment::ShoreFollowingExperiment():BehaviorController()/*:_name(nm)*/{
	_name = "uri_shorefollowing::ShoreFollowingExperiment";
}




TaskOutput ShoreFollowingExperiment::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Takeoff");
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Hover");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("ShoreFollowing");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("ShoreFollowing")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Land");
		}
	}

	
}


void ShoreFollowingExperiment::get_mandatory_resources(ResourceVector &res){
	

}


};


