#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_shorefollowing/behavior_controllers/circlingexperiment.hpp>

namespace uri_shorefollowing{

  
  
  

CirclingExperiment::CirclingExperiment():BehaviorController()/*:_name(nm)*/{
	_name = "uri_shorefollowing::CirclingExperiment";
}




TaskOutput CirclingExperiment::__run(){
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		

		_next_active_behavior = behavior("Takeoff");
	}
	
	// exit from behavior takeoff only when requested by such behavior
	else if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Hover");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	else if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Circling");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	else if (_active_behavior == behavior("Circling")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Land");
		}
	}

	
}


void CirclingExperiment::get_mandatory_resources(ResourceVector &res){


}


};


