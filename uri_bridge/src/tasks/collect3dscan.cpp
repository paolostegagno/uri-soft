#include <iostream>
#include <fstream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_bridge/tasks/collect3dscan.hpp>

namespace uri_bridge{

  
  
  

Collect3DScan::Collect3DScan():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_bridge::Collect3DScan";
	
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
}

TaskOutput Collect3DScan::_run(){
	// do your cool code here!
	
	// compute elapsed time since beginning and delta_t since last successful call
	bool terminate = false;
	double elapsed = (ros::Time::now() - start_t).toSec();
	
	delta_t = elapsed - last_elapsed;
	
	// check if new laser scan is available. If not, terminate the execution of this _run 
	if (!ls->new_laser_available()){
		return uri::Continue;
	}
	// try to retrieve the scan. If the laser scanner is busy and does not respond in 0.001 seconds, terminate the execution of this _run
	sensor_msgs::LaserScan scan;

	if (!ls->get(scan, 0.001)){
		return uri::Continue;
	}
	
	
	// integrate position
	_position[0] += delta_t*_velocity_lin(0);
	_position[1] += delta_t*_velocity_lin(1);
	_position[2] += delta_t*_velocity_lin(2);
	
	// integrate yaw
	double r, p, y;
	uri_base::quaternion_to_rpy(_orientation, r, p, y);
	y +=  delta_t*_velocity_ang(2);
	while (y >  M_PI) y -= 2*M_PI;
	while (y < -M_PI) y += 2*M_PI;
	_orientation = uri_base::rpy_to_quaternion(r, p, y);
	
	
	int num_readings = scan.ranges.size();
	double angle = scan.angle_min;
	double x_p, y_p;
	double range;
		
	
	fout << _position[0] << " " << _position[1] << " " << _position[2] 
					<< " " << r << " " << p << " " << y << " " << scan.header.seq ;
	
	for(unsigned int i = 0; i < num_readings; ++i){
		
		range = scan.ranges[i]; // traslate polar coordinates into cartesian coordinates
		x_p = range * std::cos(angle);
		y_p = range * std::sin(angle);
		
		angle += scan.angle_increment;
		fout << " " << x_p << " " << y_p; 
	}
	
	fout << std::endl;
	
	
	// update last time we integrated the pose
	last_elapsed = elapsed;

	if (terminate){
		return uri::Terminate;
	}
	return uri::Continue;
	// how to get the value of an option - use the following syntax:
	// _options["OPTION_NAME"]->getTYPEValue()
	//
	// example:
	// _options["name_option_2"]->getDoubleValue();
	//
}

void Collect3DScan::_activate(){
	// what do you need to do every time the task is activated?
	// what do you need to do every time the task is activated?
	last_elapsed = 0.0;
	start_t = ros::Time::now();
	
	fout.open ("/home/paolos/scans.txt", std::fstream::out );
	
	// select first desired heading as the current yaw
// 	double current_yaw;
// 	Eigen::Quaterniond current_ori = uav->orientation();
// 	uri_base::quaternion_to_yaw(current_ori, current_yaw);
	
}

void Collect3DScan::_deactivate(){
	// what do you need to do every time the task is deactivated?
}





void Collect3DScan::_local_position_pose_CB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	_position(0)=msg->pose.position.x;
	_position(1)=msg->pose.position.y;
	_position(2)=msg->pose.position.z;
	_orientation.w() = msg->pose.orientation.w;
	_orientation.x() = msg->pose.orientation.x;
	_orientation.y() = msg->pose.orientation.y;
	_orientation.z() = msg->pose.orientation.z;
}


void Collect3DScan::_local_position_velocity_CB(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	_velocity_lin(0)=msg->twist.linear.x;
	_velocity_lin(1)=msg->twist.linear.y;
	_velocity_lin(2)=msg->twist.linear.z;
	_velocity_ang(0)=msg->twist.angular.x;
	_velocity_ang(1)=msg->twist.angular.y;
	_velocity_ang(2)=msg->twist.angular.z;
}




void Collect3DScan::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	std::string lint("uri_sensors::LaserScanner");
	ls = (uri_sensors::LaserScanner*)res.get_resource_ptr(lint);
	
	std::string mint("uri_uav::IrisInterface");
	uav = (uri_uav::IrisInterface*)res.get_resource_ptr(mint);
	
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


