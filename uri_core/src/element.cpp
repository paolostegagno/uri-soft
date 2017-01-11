#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/element.hpp>

namespace uri{

  

Element::Element(){
	
}


Element::Element(ros::NodeHandle &_n){
	n=&_n;
}


std::string& Element::name(){
	return _name;
}

void Element::init(ros::NodeHandle &nh, TiXmlAttribute* attribute){
	

	// first read all the options and stores them in _options
	int i=0;
	if ( !attribute) {
	}
	else {
		int ival;
		double dval;
		while (attribute)
		{
			_options.updateOption(attribute);
			i++;
			attribute=attribute->Next();
		}
	}
	
	// then take the nodehandle
	n = new ros::NodeHandle(nh);
	
	
	
	_init();
}




bool Element::set_option_double(std::string &oname, double value){
	std::map<std::string,Option*>::iterator it;
	it = _options.find(oname);
	if ( it == _options.end() ) {
		ROS_INFO("WARNING: Trying to set non-existing %s option.", oname.c_str());
		return false;
	}
	((OptionDouble*)it->second)->value = value;
	return true;
}

bool Element::set_option_bool(std::string &oname, bool value){
	std::map<std::string,Option*>::iterator it;
	it = _options.find(oname);
	if ( it == _options.end() ) {
		ROS_INFO("WARNING: Trying to set non-existing %s option.", oname.c_str());
		return false;
	}
	((OptionBool*)it->second)->value = value;
	return true;
}

bool Element::set_option_string(std::string &oname, std::string value){
	std::map<std::string,Option*>::iterator it;
	it = _options.find(oname);
	if ( it == _options.end() ) {
		ROS_INFO("WARNING: Trying to set non-existing %s option.", oname.c_str());
		return false;
	}
	((OptionString*)it->second)->value = value;
	return true;
}

bool Element::set_option_int(std::string &oname, int value){
	std::map<std::string,Option*>::iterator it;
	it = _options.find(oname);
	if ( it == _options.end() ) {
		ROS_INFO("WARNING: Trying to set non-existing %s option.", oname.c_str());
		return false;
	}
	((OptionInt*)it->second)->value = value;
	return true;
}




};


