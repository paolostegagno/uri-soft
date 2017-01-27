#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_uav/tasks/attitudethrustcontroller.hpp>

namespace uri_uav{

  
  
  

AttitudeThrustController::AttitudeThrustController():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_uav::AttitudeThrustController";
	
	// The class Task contains a field _options which you can use freely.
	// You can add (and pass options through the configuration file) with the following lines.
	// [ format: _options.addTYPEOption(OPTION_NAME, DEFAULT_VALUE); ]
	//
	// _options.addIntOption("name_option_1",15);
	// _options.addDoubleOption("name_option_2",0.05);
	// _options.addBoolOption("name_option_3",false);
	// _options.addStringOption("name_option_2","default_value");
	//
	// Note that options are only of those types listed here.
	// The first parameter in the above lines is the name, while the second parameter is the default value.
	// If the option is specified in the config file, the value of the option is automatically updated.
	// If the option specfied is not in the config_file, the option will have the default value
	// note that the options are updated after the execution of this constructor, so any option in this
	// construction will have its default value.
		ofile.open("/home/paolos/savefile.txt", std::ios_base::out);
}

TaskOutput AttitudeThrustController::_run(){
	// do your cool code here!
	
	ros::Time current = ros::Time::now();
	double delta_t = (current-previous).toSec();
	
	
	double mass=1.0, g=-9.81;
	double kpx = 4.0, kvx = 1.50, kix = 0.5;
	double kpy = 4.0, kvy = 1.50, kiy = 0.5;
	double kpz = 0.5, kvz = 0.5, kiz = 0.02;
	double integral_error_bound = 3.0;
	
	
	
	
	
	Trajectory traj;
	trajectory->get(traj, 0.001);
	
	if (!trajectory->ever_set()){
		return uri::Continue;
	}
	
	
	Eigen::Vector3d pos = uav->position();
	Eigen::Vector3d vel = uav->velocity_linear();
	Eigen::Quaterniond ori = uav->orientation();
	double roll, pitch, yaw;
	uri_base::quaternion_to_rpy(ori, roll, pitch, yaw);
	
	
	
	
	Eigen::Vector3d e = traj.pos - pos;
	
	integral_error = integral_error + e*delta_t;
	
	integral_error(0) = std::max(std::min(integral_error(0), integral_error_bound), -integral_error_bound);
	integral_error(1) = std::max(std::min(integral_error(1), integral_error_bound), -integral_error_bound);
	integral_error(2) = std::max(std::min(integral_error(2), integral_error_bound), -integral_error_bound);
	
	double throttle = mass /(cos(roll)*cos(pitch))*(0.5 - kvz*vel(2) + kpz*e(2) + kiz*integral_error(2));
	
	double throttle_phys = mass /(cos(roll)*cos(pitch))*(-g - kvz*vel(2) + kpz*e(2) + kiz*integral_error(2));

	
	double roll_d = mass/throttle_phys*( sin(yaw)*(-kvx*vel(0) +kpx*e(0) + kix*integral_error(0)) - cos(yaw)*(-kvy*vel(1) +kpy*e(1) + kiy*integral_error(1)) );
	double pitch_d = mass/throttle_phys*( cos(yaw)/cos(roll)*(-kvx*vel(0) +kpx*e(0) + kix*integral_error(0) ) + sin(yaw)/cos(roll)*(-kvy*vel(1) +kpy*e(1) + kiy*integral_error(1) ) );
	double yaw_d = traj.yaw;
	
	ofile << ros::Time::now().toSec() << " " << pos(0) << " " << pos(1) << " " << pos(2) << " " << traj.pos(0) << " " << traj.pos(1) << " " << traj.pos(2) << " "
// 																		<< integral_error(0) << " " << integral_error(1) << " " << integral_error(2) << " "
																		<< e(0) << " " << e(1) << " " << e(2) << " " << traj.yaw << " " << yaw << " " << traj.yaw - yaw << std::endl;
	
	Eigen::Quaterniond ori_d = uri_base::rpy_to_quaternion(roll_d, pitch_d, yaw_d);
	
	uav->commandAttitudeThrottle(throttle, ori_d);
	
	previous = current;
	
	return uri::Continue;
}

void AttitudeThrustController::_activate(){
	// what do you need to do every time the task is activated?
	
	ros::Time previous = ros::Time::now();
		
	integral_error(0) = 0.0;
	integral_error(1) = 0.0;
	integral_error(2) = 0.0;	
}

void AttitudeThrustController::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void AttitudeThrustController::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	// to get the resources needed in this task, use the following method:
	std::string iint("uri_uav::IrisInterface");
	uav = (IrisInterface*)res.get_resource_ptr(iint);
	
	iint=std::string("uri_base::SharedMemory<uri_base::Trajectory>");
	trajectory = (uri_base::SharedMemory<uri_base::Trajectory>*)res.get_resource_ptr(iint);
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


