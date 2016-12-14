
#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_uav/iris_interface.hpp>

#ifndef __TAKEOFF_HPP__
#define __TAKEOFF_HPP__



namespace uri_uav{
	
	enum TakeoffStages{
		GROUND_START,
		GROUND_PREARM,
		GROUND_ARMING,
		GROUND_ARMED,
		TAKEOFF_START
	};

	
	class Takeoff: public Task{
		
	private:
		
		IrisInterface* uav;
		
		virtual TaskOutput _run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){
			_stage = GROUND_START;
		}
		
		virtual void _deactivate(){}

		
		TakeoffStages _stage;
		
	public:
		
		Takeoff();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_uav::Takeoff, uri::Task)
	
	
};


#endif