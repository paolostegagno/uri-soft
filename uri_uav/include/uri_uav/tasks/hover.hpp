
#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_uav/resources/iris_interface.hpp>
#include <uri_base/trajectory.hpp>
#include <uri_base/shared_memory.hpp>


#ifndef __HOVER_HPP__
#define __HOVER_HPP__



namespace uri_uav{
	
	class Hover: public Task{
		
	private:
		
		uri_uav_resources::IrisInterface* uav;
		uri_base::SharedMemory<uri_base::Trajectory>* trajectory;
		
		virtual TaskOutput _run();
		
		virtual void _initialize(){}
		
		virtual void _activate();
		
		virtual void _reset();
		
		virtual void _deactivate(){}
		
		bool _first_run;
		
		
		
		Eigen::Vector3d _goal_pos;
		Eigen::Vector3d _pos;
		Eigen::Vector3d _vel;
		Eigen::Vector3d _acc;
		
		double _goal_yaw;
		double _yaw;
		double _yawrate;
		
		
		
		Eigen::Vector3d _pos_s;
		double _yaw_s;
		ros::Time _time_start;
		ros::Time _prev_time;
		
		
	public:
		
		Hover();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

	  PLUGINLIB_EXPORT_CLASS(uri_uav::Hover, uri::Task)

	
};


#endif