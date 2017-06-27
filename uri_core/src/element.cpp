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

	/// @brief Gets a double option.
	bool Element::option(std::string name, double &value){
		auto search = _options.find(name);
		if(search != _options.end()) {
// 			std::cout << "Found " << search->first << " " << search->second << '\n';
			value = search->second->getDoubleValue();
			return true;
		}
		else {
// 			std::cout << "Not found\n";
			return false;
		}
	}
	
	
	
	bool Element::option(std::string name, std::string &value){
		auto search = _options.find(name);
		if(search != _options.end()) {
// 			std::cout << "Found " << search->first << " " << search->second << '\n';
			value = search->second->getStringValue();
			return true;
		}
		else {
// 			std::cout << "Not found\n";
			return false;
		}
	}


	bool Element::option(std::string name, bool &value){
		auto search = _options.find(name);
		if(search != _options.end()) {
// 			std::cout << "Found " << search->first << " " << search->second << '\n';
			value = search->second->getBoolValue();
			return true;
		}
		else {
// 			std::cout << "Not found\n";
			return false;
		}
	}
	
	bool Element::option(std::string name, int &value){
		auto search = _options.find(name);
		if(search != _options.end()) {
// 			std::cout << "Found " << search->first << " " << search->second << '\n';
			value = search->second->getIntValue();
			return true;
		}
		else {
// 			std::cout << "Not found\n";
			return false;
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
		/// @brief Gets a double option.
	bool Element::g_option(std::string name, double &value){
		auto search = _global_options->find(name);
		if(search != _global_options->end()) {
			value = search->second->getDoubleValue();
			return true;
		}
		else {
			return false;
		}
	}
	
	
	
	bool Element::g_option(std::string name, std::string &value){
		auto search = _global_options->find(name);
		if(search != _global_options->end()) {
// 			std::cout << "Found " << search->first << " " << search->second << '\n';
			value = search->second->getStringValue();
			return true;
		}
		else {
// 			std::cout << "Not found\n";
			return false;
		}
	}


	bool Element::g_option(std::string name, bool &value){
		auto search = _global_options->find(name);
		if(search != _global_options->end()) {
// 			std::cout << "Found " << search->first << " " << search->second << '\n';
			value = search->second->getBoolValue();
			return true;
		}
		else {
// 			std::cout << "Not found\n";
			return false;
		}
	}
	
	bool Element::g_option(std::string name, int &value){
		auto search = _global_options->find(name);
		if(search != _global_options->end()) {
// 			std::cout << "Found " << search->first << " " << search->second << '\n';
			value = search->second->getIntValue();
			return true;
		}
		else {
// 			std::cout << "Not found\n";
			return false;
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	/// @brief Sets the global options.
bool Element::set_global_option_vector_pointer(OptionVector* _govp){

	_global_options = _govp;
}
	
	

};


