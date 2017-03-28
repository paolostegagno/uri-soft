#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_base/keyboard.hpp>

#include <gazebo_msgs/ModelState.h>

#ifndef __URI_DIRECTPOSECOMMANDGZ_HPP__
#define __URI_DIRECTPOSECOMMANDGZ_HPP__



using namespace uri;
using namespace uri_base;


namespace uri_gazebo{

	/// @brief This class is an example on how to implement a Task in uri.
	/// @details This class is derived from the class uri::Task. Any uri task must provide the method belonging to this class.
	/// uri users are encouraged to use this class as base to create nrew tasks. An automatic tool may be implemented in the future.
	class DirectPoseCommandGZ: public Task{
		
		// ################ put here your private variables.
		//
		// int _variable1;
		// double _variable2;
		Keyboard* keyboard;
		
		ros::Publisher pose_publisher;
		
		Eigen::Vector3d _position;
		Eigen::Quaterniond _orientation;
		double _yaw;
		Eigen::Vector3d _velocity_lin;
		Eigen::Vector3d _velocity_ang;
		
		
		
		// ################ put here your the declaration of your private methods.
		//
		// void _cool_method(int input1, double input2);
		
		
		// ################ the following are mandatory methods that must be implemented for any task.
		//
		/// @brief Mandatory method containing the routine of the task.
		/// @return TaskOutput::Continue if the task terminates regularly, TaskOutput::Terminate if the task ask for the termination of the behavior.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual TaskOutput _run();
		
		/// @brief Mandatory method containing the routine executed ony once at the beginning.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual void _initialize();
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is activated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual void _activate();
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is deactivated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual void _deactivate();
		
	public:
		
		/// @brief Mandatory constructor.
		DirectPoseCommandGZ();
		
		/// @brief get here the resources needed in the task from the ResourceVector.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		/// @param[in] &res ResourceVector a vector containing the pointer to all the resources instantiated by the program.
		void get_mandatory_resources(ResourceVector &res);
		
	};
	
	PLUGINLIB_EXPORT_CLASS(uri_gazebo::DirectPoseCommandGZ, uri::Task)
	
	
};


#endif
