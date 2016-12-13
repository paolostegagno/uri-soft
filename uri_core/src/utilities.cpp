#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

namespace uri{
	
	void ERROR(){
	}

	void WARNING(std::string msg){
	}
	
	
	
	bool getStringParam(ros::NodeHandle &nh, std::string &s, std::string param_name, std::string robot_name){
	std::stringstream param_name_ss;
	param_name_ss << robot_name << "/" << param_name;
	if (nh.getParam(param_name_ss.str(), s))
	{
		ROS_INFO("Got param %s: %s", param_name_ss.str().c_str(), s.c_str());
		return true;
	}
	else
	{
		ROS_ERROR("Failed to get param %s", param_name_ss.str().c_str());
		return false;
	}
}

bool getBoolParam(ros::NodeHandle &nh, bool &s, std::string param_name, std::string robot_name){
	std::stringstream param_name_ss;
	param_name_ss << robot_name << "/" << param_name;
	if (nh.getParam(param_name_ss.str(), s))
	{
		ROS_INFO("Got param %s: %d", param_name_ss.str().c_str(), s);
		return true;
	}
	else
	{
		ROS_ERROR("Failed to get param %s", param_name_ss.str().c_str());
		return false;
	}
}

bool getIntParam(ros::NodeHandle &nh, int &s, std::string param_name, std::string robot_name){
	std::stringstream param_name_ss;
	param_name_ss << robot_name << "/" << param_name;
	if (nh.getParam(param_name_ss.str(), s))
	{
		ROS_INFO("Got param %s: %d", param_name_ss.str().c_str(), s);
		return true;
	}
	else
	{
		ROS_ERROR("Failed to get param %s", param_name_ss.str().c_str());
		return false;
	}
}

bool getDoubleParam(ros::NodeHandle &nh, double &s, std::string param_name, std::string robot_name){
	std::stringstream param_name_ss;
	param_name_ss << robot_name << "/" << param_name;
	if (nh.getParam(param_name_ss.str(), s))
	{
		ROS_INFO("Got param %s: %f", param_name_ss.str().c_str(), s);
		return true;
	}
	else
	{
		ROS_ERROR("Failed to get param %s", param_name_ss.str().c_str());
		return false;
	}
}



	
	
}

