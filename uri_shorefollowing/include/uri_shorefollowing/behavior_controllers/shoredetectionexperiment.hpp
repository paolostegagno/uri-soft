
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __SHOREDETECTIONEXPERIMENT_HPP__
#define __SHOREDETECTIONEXPERIMENT_HPP__

using namespace uri;

namespace uri_shorefollowing{
	
	class ShoreDetectionExperiment: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		ShoreDetectionExperiment();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_shorefollowing::ShoreDetectionExperiment, uri::BehaviorController)
	
	
};


#endif
