#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_gazebo/behavior_controllers/commandposebc.hpp>

namespace uri_gazebo{

  
  
  

CommandPoseBC::CommandPoseBC():BehaviorController()/*:_name(nm)*/{
	_name = "uri_gazebo::CommandPoseBC";
}




TaskOutput CommandPoseBC::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Viewer");
	}
	return uri::Continue;
}


void CommandPoseBC::get_mandatory_resources(ResourceVector &res){
	

}


};


