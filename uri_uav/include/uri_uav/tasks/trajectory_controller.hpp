#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

#include <uri_uav/resources/iris_interface.hpp>
#include <uri_base/trajectory.hpp>

#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>
#include <uri_base/shared_memory.hpp>


#include <fstream>

#ifndef __TRAJECTORYCONTROLLERTASK_HPP__
#define __TRAJECTORYCONTROLLERTASK_HPP__



namespace uri_uav{

	
	class PID{
	public:
		
		PID(){
			integral=0;
			first_run = true;
		}
		
		PID(double i, double p, double d){
			ki=i;
			kp=p;
			kd=d;
			integral=0;
			first_run = true;
		}
		
		double ki;
		double kp;
		double kd;
		
		double val;
		
		double integral;
		
		double lasttime;
		bool first_run;
		
		double run(double current, double desired, double vel, double des_vel){
			
			if (first_run){
				lasttime = ros::Time::now().toSec();
			}
			
			double timenow = ros::Time::now().toSec();
			double error = desired - current;
			double error_vel = des_vel - vel;
			
			integral = integral + error*(timenow-lasttime);
			lasttime=timenow;
			return kp*error + ki*integral + des_vel;
		}
		
	};
	
	

	/// @brief This class is an example on how to implement a Task in uri.
	/// @details This class is derived from the class uri::Task. Any uri task must provide the method belonging to this class.
	/// uri users are encouraged to use this class as base to create nrew tasks. An automatic tool may be implemented in the future.
	class TrajectoryControllerTask: public Task{
		
		// ################ put here your private variables.
		//
		// int _variable1;
		// double _variable2;
		
		PID _x_controller;
		PID _y_controller;
		PID _z_controller;
		
		ros::Time _init_time;
		
		IrisInterface* uav;
		uri_base::SharedMemory<uri_base::Trajectory>* trajectory;
		
		uri_base::Trajectory traj;
		
		bool _guided_mode_requested;
		
		std::fstream savefile;
		
		// ################ put here your the declaration of your private methods.
		//
		// void _cool_method(int input1, double input2);
		
		
		// ################ the following are mandatory methods that must be implemented for any task.
		//
		/// @brief Mandatory method containing the routine of the task.
		/// @return TaskOutput::Continue if the task terminates regularly, TaskOutput::Terminate if the task ask for the termination of the behavior.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual TaskOutput _run();
		
		virtual void _initialize(){
			_guided_mode_requested= false;
		}
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is activated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual void _activate();
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is deactivated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual void _deactivate();
		
	public:
		
		/// @brief Mandatory constructor.
		TrajectoryControllerTask();
		
		/// @brief get here the resources needed in the task from the ResourceVector.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		/// @param[in] &res ResourceVector a vector containing the pointer to all the resources instantiated by the program.
		void get_mandatory_resources(ResourceVector &res);
		
	};
	
	PLUGINLIB_EXPORT_CLASS(uri_uav::TrajectoryControllerTask, uri::Task)
	
	
};


#endif
