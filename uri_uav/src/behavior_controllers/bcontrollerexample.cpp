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
		if (_active_behavior->terminate()){
			std::string taskname("uri_uav::GotoTask");
			std::string oname("goal_x");
			behavior("Goto")->set_option_double(oname, taskname, 5.0);
			oname=std::string("goal_y");
			behavior("Goto")->set_option_double(oname, taskname, -7.0);
			_next_active_behavior = behavior("Goto");
			return Continue;
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Goto")){
		if (_active_behavior->terminate())
			_next_active_behavior = behavior("Hover");
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate()){
			std::string tname("uri_uav::GotoTask");
			std::string oname_x("goal_x");
			std::string oname_y("goal_y");
			switch(goto_number){
				case 0:
				{
					goto_number++;
					behavior("Goto")->set_option_double(oname_x, tname, 5.0);
					behavior("Goto")->set_option_double(oname_y, tname, -1.0);
					_next_active_behavior = behavior("Goto");
					break;
				}
				case 1: {
					goto_number++;
					behavior("Goto")->set_option_double(oname_x, tname, 0.0);
					behavior("Goto")->set_option_double(oname_y, tname, -1.0);
					_next_active_behavior = behavior("Goto");
					break;
				}
				case 2:{
					goto_number++;
					behavior("Goto")->set_option_double(oname_x, tname, 0.0);
					behavior("Goto")->set_option_double(oname_y, tname, 0.0);
					_next_active_behavior = behavior("Goto");
					break;
				}
				default:{
					goto_number=0;
					_next_active_behavior = behavior("Land");
					break;
				}
			}
		}
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


