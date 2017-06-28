#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_shorefollowing/behavior_controllers/offlineintensitymodeltest.hpp>

namespace uri_shorefollowing{

  
  
  

OfflineIntensityModelTest::OfflineIntensityModelTest():BehaviorController()/*:_name(nm)*/{
	_name = "uri_shorefollowing::OfflineIntensityModelTest";
}




TaskOutput OfflineIntensityModelTest::__run(){
// 	std::cout << executions() << std::endl;
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("CreateIntensityModel");
	}
	
}


void OfflineIntensityModelTest::get_mandatory_resources(ResourceVector &res){
	

}


};


