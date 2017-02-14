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
			std::string oname_x("goal_x");
			std::string oname_y("goal_y");
			std::string oname_z("goal_z");
			std::string oname_yaw("goal_yaw");
			behavior("Goto")->set_option_double(oname_x, taskname, 0.0);
			behavior("Goto")->set_option_double(oname_y, taskname, 9.0);
			behavior("Goto")->set_option_double(oname_z, taskname, 5.0);
			behavior("Goto")->set_option_double(oname_yaw, taskname, M_PI/2);
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
			std::string oname_z("goal_z");
			std::string oname_yaw("goal_yaw");
			switch(goto_number){
				case 0:
				{
					goto_number++;
					behavior("Goto")->set_option_double(oname_x, tname, 5.0);
					behavior("Goto")->set_option_double(oname_y, tname, -1.0);
					behavior("Goto")->set_option_double(oname_z, tname, 5.0);
					behavior("Goto")->set_option_double(oname_yaw, tname, 180.0);
					_next_active_behavior = behavior("Goto");
					break;
				}
				case 1: {
					goto_number++;
					behavior("Goto")->set_option_double(oname_x, tname, 0.0);
					behavior("Goto")->set_option_double(oname_y, tname, -1.0);
					behavior("Goto")->set_option_double(oname_z, tname, 3.0);
					behavior("Goto")->set_option_double(oname_yaw, tname, 280.0);
					_next_active_behavior = behavior("Goto");
					break;
				}
				case 2:{
					goto_number++;
					behavior("Goto")->set_option_double(oname_x, tname, 0.0);
					behavior("Goto")->set_option_double(oname_y, tname, 0.0);
					behavior("Goto")->set_option_double(oname_z, tname, 5.0);
					behavior("Goto")->set_option_double(oname_yaw, tname, 330.0);
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


