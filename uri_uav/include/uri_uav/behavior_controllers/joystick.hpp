
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __JOYSTICK_HPP__
#define __JOYSTICK_HPP__

using namespace uri;

namespace uri_uav{
	
	class Joystick: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		Joystick();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_uav::Joystick, uri::BehaviorController)
	
	
};


#endif