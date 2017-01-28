
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __SHOREFOLLOWINGEXPERIMENT_HPP__
#define __SHOREFOLLOWINGEXPERIMENT_HPP__

using namespace uri;

namespace uri_shorefollowing{
	
	class ShoreFollowingExperiment: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		ShoreFollowingExperiment();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_shorefollowing::ShoreFollowingExperiment, uri::BehaviorController)
	
	
};


#endif