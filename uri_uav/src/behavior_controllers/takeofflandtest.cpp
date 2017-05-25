#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/behavior_controllers/takeofflandtest.hpp>

namespace uri_uav{

  
  
  

TakeoffLandTest::TakeoffLandTest():BehaviorController()/*:_name(nm)*/{
	_name = "uri_uav::TakeoffLandTest";
	
/*	
	_set_mode = false;
	_armed = false;
	_takeoff = false;*/
}




TaskOutput TakeoffLandTest::__run(){
	

// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	
	std::cout << "ab " << _active_behavior << " nab " << _next_active_behavior << std::endl;
	
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Takeoff");
		return Continue;
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Hover");
			return Continue;
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Land");
		}
	}
	
	
// 	if (_active_behavior == behavior("Land")){
// 		if (_active_behavior->terminate())
// 			_next_active_behavior = behavior("Takeoff");
// 	}
}


void TakeoffLandTest::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);
	
}


};


