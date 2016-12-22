
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __EXAMPLE_BEHAVIOR_CONTROLLER_HPP__
#define __EXAMPLE_BEHAVIOR_CONTROLLER_HPP__

using namespace uri;

namespace uri_example{
	
	class ExampleBehaviorController: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		ExampleBehaviorController();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_example::ExampleBehaviorController, uri::BehaviorController)
	
	
};


#endif