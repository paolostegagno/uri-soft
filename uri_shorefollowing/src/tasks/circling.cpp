#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_shorefollowing/tasks/circling.hpp>

namespace uri_shorefollowing{

  
  
  

Circling::Circling():Task(){
	_name = "uri_shorefollowing::Circling";
	
	
	_options.addDoubleOption("angular_velocity",0.5);
	_options.addDoubleOption("countdown",10000.0);

	
}







TaskOutput Circling::_run(){
	
	// compute elapsed time since beginning and delta_t since last successful call
	double elapsed = (ros::Time::now() - start_t).toSec();
	delta_t = elapsed - last_elapsed;	
	
	double heading_velocity = _options["angular_velocity"]->getDoubleValue();
	
	// integrate the velocity
	heading_d.heading = heading_d.heading + heading_velocity*delta_t;
	// normalize the heading_d between -M_PI and M_PI
	while (heading_d.heading > M_PI) heading_d.heading = heading_d.heading - 2*M_PI;
	while (heading_d.heading < -M_PI) heading_d.heading = heading_d.heading + 2*M_PI;
	
	// here we set the desired heading in the shared memory - the controller will use it
	desired_heading->set(heading_d, 0.001);
	
	// update last time  we computed the heading
	last_elapsed = elapsed;
	
	// set terminate at true to communicate to the behavior controller to terminate the execution of the task.
	if (elapsed >= _options["countdown"]->getDoubleValue()){
		return uri::Terminate;
	}
	// but usually, terminate is false.
	return uri::Continue;
}

void Circling::_activate(){
	
	// what do you need to do every time the task is activated?
	last_elapsed = 0.0;
	start_t = ros::Time::now();
	
	delta_error = 0.0;
	delta_error_filter1=0.0;
	init_delta_error = false;
	
	// select first desired heading as the current yaw
	double current_yaw;
	Eigen::Quaterniond current_ori = uav->orientation();
	uri_base::quaternion_to_yaw(current_ori, current_yaw);
	heading_d.heading = current_yaw;
}

void Circling::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void Circling::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	
	std::string lint("uri_sensors::LaserScanner");
// 	ls = (uri_sensors::LaserScanner*)res.get_resource_ptr(lint);
	
	std::string iint("uri_base::SharedMemory<uri_base::Heading>");
	desired_heading = (uri_base::SharedMemory<uri_base::Heading>*)res.get_resource_ptr(iint);
	
	std::string mint("uri_uav::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(mint);
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


