
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __SHOREFOLLOWINGEXPERIMENT_2_HPP__
#define __SHOREFOLLOWINGEXPERIMENT_2_HPP__

using namespace uri;

namespace uri_shorefollowing{
	
	class ShoreFollowingExperiment_2: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
		
	public:
		
		ShoreFollowingExperiment_2();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_shorefollowing::ShoreFollowingExperiment_2, uri::BehaviorController)
	
	
};


#endif