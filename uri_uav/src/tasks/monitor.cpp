#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/monitor.hpp>

namespace uri_uav {

  
  

Monitor::Monitor():Task()/*:_name(nm)*/{
	_name = "uri_uav::Monitor";
}




TaskOutput Monitor::_run(){
	
	
	ROS_INFO("%s battery stats: %f V, %f A, %f %%", _name.c_str(), uav->battery_voltage(), uav->battery_current(), uav->battery_remaining());
	
	return Continue;
}


void Monitor::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);

	
}


};


