
#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_uav/resources/iris_interface.hpp>
#include <uri_sensors/resources/laserscannergazebo.hpp>
#include <uri_base/trajectory.hpp>
#include <uri_base/shared_memory.hpp>

#ifndef __TAKEOFF_HPP__
#define __TAKEOFF_HPP__



namespace uri_uav{

	
	class Takeoff: public Task{
		
	private:
		
// 		uri_sensors::LaserScanner* ls;
		uri_uav_resources::IrisInterface* uav;
		
		double _starting_height;
		
		double prearm_time;
		
		virtual TaskOutput _run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){
			_stage = GROUND_START;
		}
		
		virtual void _deactivate(){}
		
		
	enum TakeoffStages{
		GROUND_START,
		GROUND_PREARM,
		GROUND_ARMING,
		GROUND_ARMED,
		TAKEOFF_START
	};
	
	TakeoffStages _stage;
		
	public:
		
		Takeoff();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

	  PLUGINLIB_EXPORT_CLASS(uri_uav::Takeoff, uri::Task)

	
};


#endif
