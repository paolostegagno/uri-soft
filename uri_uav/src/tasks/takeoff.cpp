#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/takeoff.hpp>

namespace uri_uav {

  
  

Takeoff::Takeoff():Task()/*:_name(nm)*/{
	_name = "uri_uav::Takeoff";
	_options.addDoubleOption("takeoff_height",15);
	_options.addDoubleOption("takeoff_height_tollerance",0.05);
	
	_stage = GROUND_START;
	_starting_height = 0.0;
}




TaskOutput Takeoff::_run(){
	
	
	switch (_stage) {
		case GROUND_START:
			if (uav->setMode("guided")){
				uav->setRateRawSensors(20);
				uav->setRatePosition(10);
				uav->setRateExtendedStatus(1);
				_stage = GROUND_PREARM;
			}
			break;
			
		case GROUND_PREARM:
			if (not uav->guided()){
				_stage = GROUND_START;
				break;
			}
			if (uav->guided() and uav->local_position_pose_received()){
				if (uav->armThrottle()){
					_stage = GROUND_ARMING;
				}
			}
			break;
			
		case GROUND_ARMING:
			if (not uav->guided()){
				_stage = GROUND_START;
				break;
			}
			if (uav->armed()) {
				_stage = GROUND_ARMED;
			}
			break;
			
		case GROUND_ARMED:
			if (not uav->guided()){
				_stage = GROUND_START;
				break;
			}
			if (not uav->armed()){
				_stage = GROUND_PREARM;
			}
			else {
				_starting_height = uav->position()(2);
				std::cout << "AAAAAAAAAAAAAAAAAAAa " << _options["takeoff_height"]->getDoubleValue() + _starting_height << "  " << _options["takeoff_height"]->getDoubleValue() << " " <<  _starting_height << std::endl;
				if ( uav->takeoff( _options["takeoff_height"]->getDoubleValue() + _starting_height) ){
					_stage = TAKEOFF_START;
				}
			}
			break;
			
		case TAKEOFF_START:
			if (not uav->armed()){
				_stage = GROUND_PREARM;
			}
			if ( uav->position()(2) > _options["takeoff_height"]->getDoubleValue() + _starting_height - _options["takeoff_height_tollerance"]->getDoubleValue() ){
				return Terminate;
			}
			break;
			
		default:
			uav->disarmThrottle();
			_stage = GROUND_START;
			break;
	}
	
	return Continue;
}


void Takeoff::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);

	
}


};


