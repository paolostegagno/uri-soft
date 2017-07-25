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
		_next_active_behavior = behavior("IntensityModelCreator");
	}
	
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("IntensityModelCreator")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("ShoreDetection");
		}
	}
	
	
}


void OfflineIntensityModelTest::get_mandatory_resources(ResourceVector &res){
	

}


};


