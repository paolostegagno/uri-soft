#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_shorefollowing/behavior_controllers/shorefollowingexperiment_2.hpp>

namespace uri_shorefollowing{

  
  
  

ShoreFollowingExperiment_2::ShoreFollowingExperiment_2():BehaviorController()/*:_name(nm)*/{
	_name = "uri_shorefollowing::ShoreFollowingExperiment_2";
}




TaskOutput ShoreFollowingExperiment_2::__run(){
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
			switch (_active_behavior->counter()){
				case 1:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 1.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 0.0);
					_next_active_behavior = behavior("GotoAndCreateIntensityModel");
					break;
				case 2:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", -2.5);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 0.0);
					_next_active_behavior = behavior("GotoAndCreateIntensityModel");
					break;
				case 3:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 1.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 0.0);
					_next_active_behavior = behavior("GotoAndCreateIntensityModel");
					break;
				case 4:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 4.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 1.0);
					_next_active_behavior = behavior("Goto");
					break;
// 				case 5:
// 					_next_active_behavior = behavior("ShoreFollowing");
// 					break;
				case 5:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", 5.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 10.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 1.0);
					_next_active_behavior = behavior("Goto");
					break;
				case 6:
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", -4.0);
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_z", "uri_uav::GotoTask", 0.0);
					behavior("Goto")->set_option_double("goal_yaw", "uri_uav::GotoTask", 2.0);
					_next_active_behavior = behavior("Goto");
					break;
				case 7:
					_next_active_behavior = behavior("Land");
					break;
				default:
					_next_active_behavior = behavior("Hover");
					break;
			}
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Goto")){
		if (_active_behavior->terminate()){
// 			behavior("Hover")->set_option_double("goal_x", "uri_uav::Hover", -4.0);
			_next_active_behavior = behavior("Hover");
		}
	}
	
		// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("GotoAndCreateIntensityModel")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Hover");
		}
	}

	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("ShoreFollowing")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Land");
		}
	}

	
}


void ShoreFollowingExperiment_2::get_mandatory_resources(ResourceVector &res){
	

}


};


