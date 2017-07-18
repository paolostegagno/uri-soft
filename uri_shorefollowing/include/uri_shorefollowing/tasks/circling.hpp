#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_base/trajectory.hpp>
#include <uri_base/shared_memory.hpp>
#include <uri_uav/resources/iris_interface.hpp>

// #include <uri_sensors/resources/laserscanner.hpp>


#ifndef __URI_CIRCLING_HPP__
#define __URI_CIRCLING_HPP__



using namespace uri;
using namespace uri_base;


namespace uri_shorefollowing{
	
	
	
	
	
	
	
	

	/// @brief This class is an example on how to implement a Task in uri.
	/// @details This class is derived from the class uri::Task. Any uri task must provide the method belonging to this class.
	/// uri users are encouraged to use this class as base to create nrew tasks. An automatic tool may be implemented in the future.
	class Circling: public Task{
		
		// ################ put here your private variables.

		// time variables
		double delta_t;
		ros::Time start_t;
		double last_elapsed;
		
		uri_base::Heading heading_d;
		double last_current_yaw;
		
		double integral_error;
		double previous_error;
		double prev_sign;
		double delta_error_filter1;
		double delta_error;
		double init_delta_error;
		
		std::vector <Eigen::Vector2d> shore_points;
		
		// Resources
// 		uri_sensors::LaserScanner* ls;
		uri_base::SharedMemory<uri_base::Heading>* desired_heading;
		uri_uav_resources::IrisInterface* uav;
		
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
		
		/// @brief Computes the yawrate to follow the shore from a laser scan.
		/// @param[in] &scan the current laser scan in ROS format.
		/// @return The reference yawrate.
// 		double compute_heading_velocity(sensor_msgs::LaserScan &scan);
		
		
	public:
		
		/// @brief Mandatory constructor.
		Circling();
		
		/// @brief get here the resources needed in the task from the ResourceVector.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		/// @param[in] &res ResourceVector a vector containing the pointer to all the resources instantiated by the program.
		void get_mandatory_resources(ResourceVector &res);
		
	};
	
	PLUGINLIB_EXPORT_CLASS(uri_shorefollowing::Circling, uri::Task)
	
	
};


#endif
