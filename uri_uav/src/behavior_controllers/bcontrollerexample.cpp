#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/behavior_controllers/bcontrollerexample.hpp>

namespace uri_uav{

  
  
  

BControllerExample::BControllerExample():BehaviorController()/*:_name(nm)*/{
	_name = "uri_uav::BControllerExample";
	
/*	
	_set_mode = false;
	_armed = false;
	_takeoff = false;*/
}




TaskOutput BControllerExample::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Takeoff");
		return Continue;
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate())
			_next_active_behavior = behavior("Goto");
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Goto")){
		if (_active_behavior->terminate())
			_next_active_behavior = behavior("Hover");
	}
	
	
	if (_active_behavior == behavior("Land")){
		if (_active_behavior->terminate())
			_next_active_behavior = behavior("Takeoff");
	}
}


void BControllerExample::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);
	
}


};


