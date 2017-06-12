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
	
	_options.addDoubleOption("mass",1.0);
	_options.addDoubleOption("kpx",4.0);
	_options.addDoubleOption("kpy",4.0);
	_options.addDoubleOption("kpz",0.5);
	_options.addDoubleOption("kvx",1.5);
	_options.addDoubleOption("kvy",1.5);
	_options.addDoubleOption("kvz",0.5);
	_options.addDoubleOption("kix",0.5);
	_options.addDoubleOption("kiy",0.5);
	_options.addDoubleOption("kiz",0.02);
	_options.addDoubleOption("integral_error_bound_x",3.0);
	_options.addDoubleOption("integral_error_bound_y",3.0);
	_options.addDoubleOption("integral_error_bound_z",3.0);
	_options.addBoolOption("save_data", false);
	_options.addStringOption("savefile_name", "/home/paolos/savefile.txt");
	
	// Note that options are only of those types listed here.
	// The first parameter in the above lines is the name, while the second parameter is the default value.
	// If the option is specified in the config file, the value of the option is automatically updated.
	// If the option specfied is not in the config_file, the option will have the default value
	// note that the options are updated after the execution of this constructor, so any option in this
	// construction will have its default value.
	

}

TaskOutput AttitudeThrustController::_run(){
	
	// get the time since last call (for the error integration)
	ros::Time current = ros::Time::now();
	double delta_t = (current-previous).toSec();
	
	// retrieve all control gains and dynamic parameters from the options
	double mass=_options["mass"]->getDoubleValue(), g=-9.81;
	double kpx = _options["kpx"]->getDoubleValue(), kvx = _options["kvx"]->getDoubleValue(), kix = _options["kix"]->getDoubleValue();
	double kpy = _options["kpy"]->getDoubleValue(), kvy = _options["kvy"]->getDoubleValue(), kiy = _options["kiy"]->getDoubleValue();
	double kpz = _options["kpz"]->getDoubleValue(), kvz = _options["kvz"]->getDoubleValue(), kiz = _options["kiz"]->getDoubleValue();
	double integral_error_bound_x = _options["integral_error_bound_x"]->getDoubleValue();
	double integral_error_bound_y = _options["integral_error_bound_y"]->getDoubleValue();
	double integral_error_bound_z = _options["integral_error_bound_z"]->getDoubleValue();
	
	// retrieve the desired trajectory (it is a mutexed resource)
	Trajectory traj;
	trajectory->get(traj, 0.001);
	if (!trajectory->ever_set()){
		return uri::Continue;
	}
	
	// retrieve position, velocity, orientation of the uav
	Eigen::Vector3d pos = uav->position();
	Eigen::Vector3d vel = uav->velocity_linear();
	Eigen::Quaterniond ori = uav->orientation();
	double roll, pitch, yaw;
	uri_base::quaternion_to_rpy(ori, roll, pitch, yaw);
	
	// compute the position error
	Eigen::Vector3d e = traj.pos - pos;
	
	// compute the integral error and saturate it (as it may grow to infinite)
	integral_error = integral_error + e*delta_t;
	integral_error(0) = std::max(std::min(integral_error(0), integral_error_bound_x), - integral_error_bound_x );
	integral_error(1) = std::max(std::min(integral_error(1), integral_error_bound_y), - integral_error_bound_y );
	integral_error(2) = std::max(std::min(integral_error(2), integral_error_bound_z), - integral_error_bound_z );
	
	// apply near hovering control law.
	// NOTE: some firmware developer thought it was a good idea to change the thrust command that I give to the quadcopter.
	// although the physically correct line is in throttle_phys - which I have to compute to use it in the computation of roll_d and pitch_d,
	// the command that is actually given is throttle, where -g is replaced with 0.5.
	// If you open this file and want to change something, keep it in mind and deal with it...
	double throttle = mass /(cos(roll)*cos(pitch))*(0.5 - kvz*vel(2) + kpz*e(2) + kiz*integral_error(2));
	double throttle_phys = mass /(cos(roll)*cos(pitch))*(-g - kvz*vel(2) + kpz*e(2) + kiz*integral_error(2));
	
	double roll_d = mass/throttle_phys*( sin(yaw)*(-kvx*vel(0) +kpx*e(0) + kix*integral_error(0)) - cos(yaw)*(-kvy*vel(1) +kpy*e(1) + kiy*integral_error(1)) );
	double pitch_d = mass/throttle_phys*( cos(yaw)/cos(roll)*(-kvx*vel(0) +kpx*e(0) + kix*integral_error(0) ) + sin(yaw)/cos(roll)*(-kvy*vel(1) +kpy*e(1) + kiy*integral_error(1) ) );
	double yaw_d = traj.yaw;
	
// 	std::cout << yaw_d << std::endl;
	
	// save the interesting quantities in an output file to plot the result
	if (_options["save_data"]->getBoolValue()){
double time_now = ros::Time::now().toSec();
		ofile << time_now << " " << pos(0) << " " << pos(1) << " " << pos(2) << " " << traj.pos(0) << " " << traj.pos(1) << " " << traj.pos(2) << " "
																		<< vel(0) << " " << vel(1) << " " << vel(2) << " "
																		<< integral_error(0) << " " << integral_error(1) << " " << integral_error(2) << " "
																		<< e(0) << " " << e(1) << " " << e(2) << " " << traj.yaw << " " << yaw << " " << traj.yaw - yaw << std::endl;
	}
	
	// creat a quaternion oit of the desired roll, pitch and yaw
	Eigen::Quaterniond ori_d = uri_base::rpy_to_quaternion(roll_d, pitch_d, yaw_d);
	
	// apply the command
	uav->commandAttitudeThrottle(throttle, ori_d);
	
	// save the current timestamp for the next call of this function
	previous = current;
	
	return uri::Continue;
}



void AttitudeThrustController::_activate(){
	// what do you need to do every time the task is activated?
	
	
	if (_options["save_data"]->getBoolValue()){
		ofile.open(_options["savefile_name"]->getStringValue(), std::ios_base::out);
	}
	
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


