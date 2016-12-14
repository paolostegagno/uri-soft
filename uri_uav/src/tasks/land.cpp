#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/land.hpp>

namespace uri_uav {

  
  

Land::Land():Task()/*:_name(nm)*/{
	_name = "uri_uav::Land";
	_options.addDoubleOption("land_height",0.3);
	_options.addDoubleOption("land_speed",-0.4);
	
	_stage = LAND_START;
}




TaskOutput Land::_run(){
	
	switch (_stage) {
		case LAND_START:
			if (uav->setMode("guided")){
				_stage = LAND_PREDESCEND;
			}
			break;
			
		case LAND_PREDESCEND:
			if (not uav->guided()){
				_stage = LAND_START;
				break;
			}
			else {
				_stage = LAND_DESCENDING;
			}
			break;
			
			
		case LAND_DESCENDING:
			uav->commandVelocity(0.0, 0.0, _options["land_speed"]->getDoubleValue());
			if ( uav->position()(2) < _options["land_height"]->getDoubleValue() ){
				_stage = LAND_GROUND;
			}
			break;
			
		case LAND_GROUND:
			if ( uav->disarmThrottle() ){
				return Terminate;
			}
			break;

			
		default:
// 			uav->disarmThrottle();
			_stage = LAND_START;
			break;
	}
	
	return Continue;
}


void Land::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);
	
	iint = std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);

	
}


};


