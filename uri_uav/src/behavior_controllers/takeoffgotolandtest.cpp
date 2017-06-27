#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/behavior_controllers/takeoffgotolandtest.hpp>

namespace uri_uav{

  
  
  

TakeoffGotoLandTest::TakeoffGotoLandTest():BehaviorController()/*:_name(nm)*/{
	_name = "uri_uav::TakeoffGotoLandTest";
	
/*	
	_set_mode = false;
	_armed = false;
	_takeoff = false;*/
}




TaskOutput TakeoffGotoLandTest::__run(){
	

// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	
	
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Takeoff");
		return Continue;
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate()){
			behavior("Hover")->set_option_double("Countdown", "uri_uav::Hover", 1.0);
			_next_active_behavior = behavior("Hover");
			goto_number=0;
			return Continue;
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate()){
			switch (goto_number){
				case 0:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 0.0);
					break;
				case 1:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 1.0);
					break;
				case 2:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 2.0);
					break;
				case 3:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 3.0);
					break;
				case 4:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 4.0);
					break;
				case 5:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 5.0);
					break;
				case 6:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 6.0);
					break;
				case 7:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 0.0);
					break;
				case 8:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", -3.0);
					break;
				default: break;
			}
			if (goto_number==9){
				_next_active_behavior = behavior("Land");
			}
			else {
				_next_active_behavior = behavior("Goto");
			}
		}
	}
	
	
	if (_active_behavior == behavior("Goto")){
		if (_active_behavior->terminate()){
			goto_number = goto_number+1;
			_next_active_behavior = behavior("Hover");
		}
	}
	
	
// 	if (_active_behavior == behavior("Land")){
// 		if (_active_behavior->terminate())
// 			_next_active_behavior = behavior("Takeoff");
// 	}
}


void TakeoffGotoLandTest::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav_resources::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(iint);
	
}


};


