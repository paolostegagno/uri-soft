#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_shorefollowing/behavior_controllers/shoredetectionexperiment.hpp>

namespace uri_shorefollowing{

  
  
  

ShoreDetectionExperiment::ShoreDetectionExperiment():BehaviorController()/*:_name(nm)*/{
	_name = "uri_shorefollowing::ShoreDetectionExperiment";
}




TaskOutput ShoreDetectionExperiment::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("ShoreDetection");
	}	
}


void ShoreDetectionExperiment::get_mandatory_resources(ResourceVector &res){
	

}


};


