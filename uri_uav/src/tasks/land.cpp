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
	_options.addDoubleOption("land_acceleration",-0.2);
	_options.addDoubleOption("land_detection_zero_velocity_threshold",-0.1); // i.e., vertical velocity vz is considered zero if  land_detection_zero_velocity_threshold < vy < -land_detection_zero_velocity_threshold
	_options.addDoubleOption("land_detection_not_confirmed_land_duration",1.0); // time on the ground to confirm landing
	
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
			_prev_time = ros::Time::now();
			traj.pos = uav->position();
			traj.vel << 0,0,0;
			traj.acc << 0,0,_options["land_acceleration"]->getDoubleValue();
			traj.yawrate = 0.0;
			traj.yaw = uav->yaw();
			
			if (not uav->guided()){
				_stage = LAND_START;
				break;
			}
			else {
				_stage = LAND_DESCENDING;
			}
			break;
			
			
		case LAND_DESCENDING: {
				double delta_t ;
				ros::Time _time_now = ros::Time::now();
				delta_t = (_time_now - _prev_time).toSec();
				
				if ( traj.vel(2) > _options["land_speed"]->getDoubleValue() ) {
					traj.vel = traj.vel + delta_t*traj.acc;
					traj.pos = traj.pos + delta_t*traj.vel;
				}
				else {
					traj.pos = traj.pos + delta_t*traj.vel;
					traj.acc(2) = 0;
				}
				_prev_time = _time_now;
				trajectory->set(traj,0.01);
				std::cout << "LAND_DESCENDING " << _time_now.toSec() << " " << uav->position()(2) << std::endl;
				
				if (uav->velocity_linear()(2)> _options["land_detection_zero_velocity_threshold"]->getDoubleValue() && uav->velocity_linear()(2)< -_options["land_detection_zero_velocity_threshold"]->getDoubleValue() ){
					if (uav->velocity_linear()(2)-traj.vel(2) > -_options["land_speed"]->getDoubleValue()*0.7 ) {
						_landing_detected = ros::Time::now();
						_stage = LAND_NOT_CONFIRMED_GROUND;
					}
				}
			break;
		}
		
		case LAND_NOT_CONFIRMED_GROUND:
			std::cout << "LAND_NOT_CONFIRMED_GROUND " << uav->position()(2) << " " << (ros::Time::now() - _landing_detected).toSec() << std::endl;
			if (uav->velocity_linear()(2)> _options["land_detection_zero_velocity_threshold"]->getDoubleValue() && uav->velocity_linear()(2)< -_options["land_detection_zero_velocity_threshold"]->getDoubleValue() ){
				if (uav->velocity_linear()(2)-traj.vel(2) > -_options["land_speed"]->getDoubleValue()*0.7 ) {
					if ((ros::Time::now() - _landing_detected).toSec()>_options["land_detection_not_confirmed_land_duration"]->getDoubleValue())
					_stage = LAND_GROUND;
				}
			}
			break;
			
			
		case LAND_GROUND:
			std::cout << "LAND_GROUND " << uav->position()(2) << std::endl;
			uav->disarmThrottle();
			if (not uav->armed()) {
				_stage = LAND_GROUND_DISARMED;
				return Continue;
			}
			break;
			
		case LAND_GROUND_DISARMED:
			std::cout << "LAND_GROUND DISARMED" << std::endl;
			return Terminate;
			break;
			
		default:
// 			uav->disarmThrottle();
			_stage = LAND_START;
			break;
	}
	
	return Continue;
}


void Land::get_mandatory_resources(ResourceVector &res){
	
	std::string iint("uri_uav_resources::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(iint);
	
	iint = std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);

	
}


};


