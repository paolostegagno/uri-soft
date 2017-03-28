
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __COMMANDPOSEBC_HPP__
#define __COMMANDPOSEBC_HPP__

using namespace uri;

namespace uri_gazebo{
	
	class CommandPoseBC: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		CommandPoseBC();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_gazebo::CommandPoseBC, uri::BehaviorController)
	
	
};


#endif