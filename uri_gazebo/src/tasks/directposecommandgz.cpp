#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_gazebo/tasks/directposecommandgz.hpp>
#include <uri_base/angle_conversion.hpp>



namespace uri_gazebo{

  
  
  

DirectPoseCommandGZ::DirectPoseCommandGZ():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_gazebo::DirectPoseCommandGZ";
	
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

TaskOutput DirectPoseCommandGZ::_run(){
	// do your cool code here!
	
	bool terminate = false;
	char* chars;
	
	int numchar = keyboard->get(chars);
	
	if (numchar>0){
		std::cout << " read " << numchar << " chars:";
		for (int i =0; i<numchar; i++){
			std::cout << " " << chars[i];
			switch (chars[i]){
				case 'q': terminate = true; break;
				case 'a': _yaw-=0.1; break;
				case 'd': _yaw+=0.1; break;
				case 'w': _position(2)+=0.1; break;
				case 's': _position(2)-=0.1; break;
				case 'i': _position(0)+=0.1 ; break;
				case 'k': _position(0)-=0.1 ; break;
				case 'j': _position(1)+=0.1; break;
				case 'l': _position(1)-=0.1; break;
				default: break;
			}
		}
		std::cout << std::endl;
		
		delete chars;
	}
	
	
	
	_orientation = rpy_to_quaternion(0, 0, _yaw);
	
	gazebo_msgs::ModelState setModelState;
	
	setModelState.model_name = "iris";
	setModelState.pose.position.x = _position[0];// = getModelState.response.pose;
	setModelState.pose.position.y = _position[1];// = getModelState.response.pose;
	setModelState.pose.position.z = _position[2]+0.25;// = getModelState.response.pose;
	setModelState.pose.orientation.x = _orientation.x();// = getModelState.response.pose;
	setModelState.pose.orientation.y = _orientation.y();// = getModelState.response.pose;
	setModelState.pose.orientation.z = _orientation.z();// = getModelState.response.pose;
	setModelState.pose.orientation.w = _orientation.w();// = getModelState.response.pose;
	setModelState.twist.linear.x = _velocity_lin(0);
	setModelState.twist.linear.y = _velocity_lin(1);
	setModelState.twist.linear.z = _velocity_lin(2);
	setModelState.twist.angular.z = _velocity_ang(2);
	
	pose_publisher.publish<gazebo_msgs::ModelState>(setModelState);
	
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

void DirectPoseCommandGZ::_initialize(){
	pose_publisher = n->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 0);
}


void DirectPoseCommandGZ::_activate(){
	_position(0) = 0.0;
	_position(1) = 0.0;
	_position(2) = 0.0;
	_orientation.x() = 0;
	_orientation.y() = 0;
	_orientation.z() = 0;
	_orientation.w() = 1;
	_velocity_lin(0) = 0.0;
	_velocity_lin(1) = 0.0;
	_velocity_lin(2) = 0.0;
	_velocity_ang(0) = 0.0;
	_velocity_ang(1) = 0.0;
	_velocity_ang(2) = 0.0;
	_yaw = 0;
	// what do you need to do every time the task is activated?
}

void DirectPoseCommandGZ::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void DirectPoseCommandGZ::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	std::string iint("uri_base::Keyboard");
	keyboard = (Keyboard*)res.get_resource_ptr(iint);
	
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


