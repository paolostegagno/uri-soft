#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <pluginlib/class_loader.h>

#include <tinyxml.h>

#include <uri_core/utilities.hpp>
#include <uri_core/resource.hpp>
#include <uri_core/task.hpp>
#include <uri_core/behavior_controller.hpp>
#include <uri_core/behavior.hpp>





#ifndef __URI_SCHEDULER_HPP__
#define __URI_SCHEDULER_HPP__



namespace uri{
	
	/// @brief This class contains the functionalities to load and peoridically run Resources, Tasks, Behaviors, and a BehaviorController.
	class Scheduler{
		
		ros::NodeHandle n;
		
		pluginlib::ClassLoader<uri::Resource> *resource_loader;
		ResourceVector resources;
		
		pluginlib::ClassLoader<uri::Task> *task_loader;
		std::vector<boost::shared_ptr<uri::Task> > tasks;
		
		pluginlib::ClassLoader<uri::BehaviorController> *behavior_controller_loader;
		boost::shared_ptr<uri::BehaviorController> behavior_controller;
		bool behavior_controller_found;
		
		std::vector<uri::Behavior*> behaviors;
		
		
	public:
		OptionVector* global_options;
		
		/// @brief Complete constructor
		/// @param[in] nh a ros::NodeHandle
		/// @param[in] config_file_name the name of the uri-soft configuration file that contains all tasks, resources, parameters, behaviors, etc.
		Scheduler(ros::NodeHandle &nh, std::string &config_file_name);
		
	private:
		
		// this function prints all the attributes of an element in an xml file
		int dump_attribs_to_stdout(TiXmlElement* pElement);
		
		// load the named file and dump its structure to STDOUT
		bool load_configuration_file(const char* pFilename, TiXmlDocument &doc);
		
		// load all the global_options in the TiXmlNode tree provided
		void load_global_options( TiXmlNode* pParent);
		
		// load all the resources in the TiXmlNode tree provided
		void load_resources( TiXmlNode* pParent);
		
		// load all the Tasks in the TiXmlNode tree provided
		void load_tasks( TiXmlNode* pParent);
		
		// load one behavior_controller in the TiXmlNode tree provided
		void load_behavior_controller( TiXmlNode* pParent);
		
		// load all the behaviors in the TiXmlNode tree provided
		void load_behaviors( TiXmlNode* pParent);
		
	};
	
};


#endif

