
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __SRTEXPLORATION_HPP__
#define __SRTEXPLORATION_HPP__

using namespace uri;

namespace uri_bridge{
	
	class SRTExploration: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		SRTExploration();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_bridge::SRTExploration, uri::BehaviorController)
	
	
};


#endif