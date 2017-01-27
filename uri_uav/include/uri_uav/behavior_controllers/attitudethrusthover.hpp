
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __ATTITUDETHRUSTHOVER_HPP__
#define __ATTITUDETHRUSTHOVER_HPP__

using namespace uri;

namespace uri_uav{
	
	class AttitudeThrustHover: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		AttitudeThrustHover();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_uav::AttitudeThrustHover, uri::BehaviorController)
	
	
};


#endif