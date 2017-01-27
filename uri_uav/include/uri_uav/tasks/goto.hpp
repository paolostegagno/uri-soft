#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_uav/resources/iris_interface.hpp>
#include <uri_base/trajectory.hpp>


#ifndef __GOTO_HPP__
#define __GOTO_HPP__



namespace uri_uav{
	
	enum GotoStages{
		GOTO_START,
		GOTO_ACCELERATION,
		GOTO_CONSTANT_SPEED,
		GOTO_DECELERATION,
		GOTO_END,
	};

	/// @brief This class is an example on how to implement a Task in uri-soft.
	/// @details This class is derived from the class uri-soft::Task. Any uri-soft task must provide the method belonging to this class.
	/// uri-soft users are encouraged to use this class as base to create nrew tasks. An automatic tool may be implemented in the future.
	class GotoTask: public Task{
		
		// ################ put here your private variables.
		GotoStages _stage;
		
		IrisInterface* uav;
		uri_base::SharedMemory<uri_base::Trajectory>* trajectory;
		
		
		
		Eigen::Vector3d _start;
		Eigen::Vector3d _startvel;
		double _start_yaw;
		ros::Time _start_time;
		
		double t1, t2, t3;
		double x1, x2;
		double y1, y2;
		double z1, z2;
		
		Eigen::Vector3d _goal;
		Eigen::Vector3d _goalvel;
		double _goal_yaw;
		
		double max_speed;
		double max_x_vel;
		double max_y_vel;
		double max_z_vel;
		
		double max_acc;
		double max_x_acc;
		double max_y_acc;
		double max_z_acc;
		
		double _yawrate;
		double max_yawrate;
		
		Eigen::Vector3d _direction;
		
		
		// ################ put here your the declaration of your private methods.
		//
		// void _cool_method(int input1, double input2);
		
		
		// ################ the following are mandatory methods that must be implemented for any task.
		//
		/// @brief Mandatory method containing the routine of the task.
		/// @return TaskOutput::Continue if the task terminates regularly, TaskOutput::Terminate if the task ask for the termination of the behavior.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri-soft::Task.
		virtual TaskOutput _run();
		
		virtual void _initialize(){
			max_speed = _options["max_vel"]->getDoubleValue();
			max_acc = _options["max_acc"]->getDoubleValue();
			_goal(0) = _options["goal_x"]->getDoubleValue();
			_goal(1) = _options["goal_y"]->getDoubleValue();
			_goal(2) = _options["goal_z"]->getDoubleValue();
		}
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is activated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri-soft::Task.
		virtual void _activate();
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is deactivated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri-soft::Task.
		virtual void _deactivate();
		
	public:
		
		/// @brief Mandatory constructor.
		GotoTask();
		
		/// @brief get here the resources needed in the task from the ResourceVector.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri-soft::Task.
		/// @param[in] &res ResourceVector a vector containing the pointer to all the resources instantiated by the program.
		void get_mandatory_resources(ResourceVector &res);
		
	};
	
	PLUGINLIB_EXPORT_CLASS(uri_uav::GotoTask, uri::Task)
	
	
};


#endif
