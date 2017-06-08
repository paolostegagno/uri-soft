
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#ifndef __CIRCLINGEXPERIMENT_HPP__
#define __CIRCLINGEXPERIMENT_HPP__

using namespace uri;

namespace uri_shorefollowing{
	
	class CirclingExperiment: public BehaviorController{
		
	private:
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){

		}
		
		virtual void _activate(){

			
		}
		
		virtual void _deactivate(){}
		
		
	public:
		
		CirclingExperiment();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_shorefollowing::CirclingExperiment, uri::BehaviorController)
	
	
};


#endif