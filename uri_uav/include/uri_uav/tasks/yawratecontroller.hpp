#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

#include <uri_uav/resources/iris_interface.hpp>
#include <uri_base/trajectory.hpp>

#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <fstream>


#ifndef __URI_YAWRATECONTROLLER_HPP__
#define __URI_EXAMPLE_TAKS_HPP__



using namespace uri;


namespace uri_uav{

	/// @brief This class is an example on how to implement a Task in uri.
	/// @details This class is derived from the class uri::Task. Any uri task must provide the method belonging to this class.
	/// uri users are encouraged to use this class as base to create nrew tasks. An automatic tool may be implemented in the future.
	class YawrateController: public Task{
		
		// ################ put here your private variables.
		//
		IrisInterface* uav;
		uri_base::SharedMemory<uri_base::Trajectory>* trajectory;
		
		uri_base::Trajectory traj;
		
		bool _guided_mode_requested;
		
		
		
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
		virtual void _initialize(){}
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is activated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual void _activate();
		
		/// @brief Mandatory method containing the routine executed ony once every time the task is deactivated.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		virtual void _deactivate();
		
	public:
		
		/// @brief Mandatory constructor.
		YawrateController();
		
		/// @brief get here the resources needed in the task from the ResourceVector.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		/// @param[in] &res ResourceVector a vector containing the pointer to all the resources instantiated by the program.
		void get_mandatory_resources(ResourceVector &res);
		
	};
	
	PLUGINLIB_EXPORT_CLASS(uri_uav::YawrateController, uri::Task)
	
	
};


#endif
