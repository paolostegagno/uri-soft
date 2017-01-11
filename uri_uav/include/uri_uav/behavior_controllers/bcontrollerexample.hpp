
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>

#include <uri_uav/resources/iris_interface.hpp>

#ifndef __BCONTROLLEREXAMPLE_HPP__
#define __BCONTROLLEREXAMPLE_HPP__



namespace uri_uav{
	
	class BControllerExample: public BehaviorController{
		
	private:
		
			int goto_number;
		
		IrisInterface* uav;
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){
			goto_number = 0;
		}
		
		virtual void _deactivate(){}
		
/*		
		bool _set_mode;
		bool _armed;
		bool _takeoff;*/
		
	public:
		
		BControllerExample();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_uav::BControllerExample, uri::BehaviorController)
	
	
};


#endif