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
	
	class Scheduler{
		
		
// 		int _active_behavior;
// 		std::string _active_behavior_name;
		
// 		int _callbacks;
		
		ros::NodeHandle n;
		
	public:
		
		pluginlib::ClassLoader<uri::Resource> *resource_loader;
		ResourceVector resources;
		
		pluginlib::ClassLoader<uri::Task> *task_loader;
		std::vector<boost::shared_ptr<uri::Task> > tasks;
		
		
		pluginlib::ClassLoader<uri::BehaviorController> *behavior_controller_loader;
		boost::shared_ptr<uri::BehaviorController> behavior_controller;
		bool behavior_controller_found;
		
		std::vector<uri::Behavior*> behaviors;
		
// 		ros::Timer timer;
		
		
		Scheduler(ros::NodeHandle &nh, std::string &config_file_name, double scheduler_time);

		
		int dump_attribs_to_stdout(TiXmlElement* pElement);
		
		// load the named file and dump its structure to STDOUT
		bool load_configuration_file(const char* pFilename, TiXmlDocument &doc);
		
		void load_resources( TiXmlNode* pParent);
		
		void load_tasks( TiXmlNode* pParent);
		
		void load_behavior_controller( TiXmlNode* pParent);
		
		void load_behaviors( TiXmlNode* pParent);
		
// 		void run(const ros::TimerEvent&);
		
		
	};
	
};


#endif

