#include <iostream>
#include <vector>
#include <boost/iterator/iterator_concepts.hpp>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>


#ifndef __URI_DUMMY_TASK_HPP__
#define __URI_DUMMY_TAKS_HPP__



using namespace uri;

namespace uri_example{
	
	class DummyTask: public Task{
		
		
		virtual TaskOutput _run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
	public:
		
		DummyTask();
		
		/// @brief Mandatory initialization method.
		void get_mandatory_resources(ResourceVector &res);
		
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_example::DummyTask, uri::Task)
	
	
};


#endif
