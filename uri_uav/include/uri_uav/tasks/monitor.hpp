
#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_uav/resources/iris_interface.hpp>

#ifndef __MONITOR_HPP__
#define __MONITOR_HPP__



namespace uri_uav{
	
	class Monitor: public Task{
		
	private:

		uri_uav_resources::IrisInterface* uav;
		
		virtual TaskOutput _run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){
		}
		
		virtual void _deactivate(){}

		
		
	public:
		
		Monitor();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_uav::Monitor, uri::Task)
	
	
};


#endif
