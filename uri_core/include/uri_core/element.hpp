#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <tinyxml.h>

#include <uri_core/utilities.hpp>


#ifndef __URI_ELEMENT_HPP__
#define __URI_ELEMENT_HPP__



namespace uri{
	
/// @brief Base class for any element to be executed in uri.
/// @details This class is the base class for any element that can run in URI.
	class Element{
		
	private:
		
		/// @brief All child classes will need to implement an init method that is called at the end of the init public method of this class
		virtual void _init()=0;
		
	protected:
		
		/// @brief Name of the element.
		std::string _name;
		
		/// @brief Container for the options of this class and its child classes.
		OptionVector _options;
		
		/// @brief pointer to a ros nodehandle
		ros::NodeHandle* n;
		
	public:
		
		/// @brief Main constructor.
		/// @details Setup two Options: period with default value 0.1 and period_tollerance with default value 0.01. Puts some values at zero.
		Element();
		
		/// @brief Main constructor.
		/// @details Setup two Options: period with default value 0.1 and period_tollerance with default value 0.01. Puts some values at zero.
		Element(ros::NodeHandle &_n);

		
		/// @brief Returns the name of the element
		/// @return std::string containing the name of the element.
		std::string& name();
		
		/// @brief Reads the options from the xml file and setup the timer to call run at constant time period
		void init(ros::NodeHandle &nh, TiXmlAttribute* attribute);
		
		/// @brief Set an option.
		/// 
		/// @return Returns \b true if the option exists and has been set, \b false otherwise.
		bool set_option_double(std::string &oname, double value){
			std::map<std::string,Option*>::iterator it;
			it = _options.find(oname);
			if ( it == _options.end() ) {
				ROS_INFO("WARNING: Trying to set non-existing %s option.", oname.c_str());
				return false;
			}
			((OptionDouble*)it->second)->value = value;
			return true;
		}
		bool set_option_bool(std::string &oname, bool value){
			std::map<std::string,Option*>::iterator it;
			it = _options.find(oname);
			if ( it == _options.end() ) {
				ROS_INFO("WARNING: Trying to set non-existing %s option.", oname.c_str());
				return false;
			}
			((OptionBool*)it->second)->value = value;
			return true;
		}
		bool set_option_string(std::string &oname, std::string value){
			std::map<std::string,Option*>::iterator it;
			it = _options.find(oname);
			if ( it == _options.end() ) {
				ROS_INFO("WARNING: Trying to set non-existing %s option.", oname.c_str());
				return false;
			}
			((OptionString*)it->second)->value = value;
			return true;
		}
		bool set_option_int(std::string &oname, int value){
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

	
	
};


#endif
