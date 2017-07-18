
#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_uav/resources/iris_interface.hpp>
#include <uri_base/trajectory.hpp>
#include <uri_base/shared_memory.hpp>

#ifndef __LAND_HPP__
#define __LAND_HPP__



namespace uri_uav{
	
	enum LandStages{
		LAND_START,
		LAND_PREDESCEND,
		LAND_DESCENDING,
		LAND_NOT_CONFIRMED_GROUND,
		LAND_GROUND,
		LAND_GROUND_DISARMED
	};

	
	class Land: public Task{
		
	private:
		
		uri_uav_resources::IrisInterface* uav;
		uri_base::SharedMemory<uri_base::Trajectory> *trajectory;
		
		virtual TaskOutput _run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){
			_stage = LAND_START;
		}
		
		virtual void _deactivate(){}
		
		LandStages _stage;
		
		ros::Time _prev_time;
		ros::Time _landing_detected;
		
		Eigen::Vector3d _first_pos;
		uri_base::Trajectory traj;
		
	public:
		
		Land();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_uav::Land, uri::Task)
	
	
};


#endif