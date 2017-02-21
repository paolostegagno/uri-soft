#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_bridge/behavior_controllers/srtexploration.hpp>

namespace uri_bridge{

  
  
  

SRTExploration::SRTExploration():BehaviorController()/*:_name(nm)*/{
	_name = "uri_bridge::SRTExploration";
}




TaskOutput SRTExploration::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Takeoff");
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Goto");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Goto")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Hover");
		}
	}
	
	if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Collect3DScan");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Collect3DScan")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Goto");
		}
	}

	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("ShoreFollowing")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Land");
		}
	}
	
}


void SRTExploration::get_mandatory_resources(ResourceVector &res){
	

}


};


