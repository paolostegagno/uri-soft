#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>

#include <pluginlib/class_list_macros.h>
#include <uri_core/task.hpp>

#include <uri_base/trajectory.hpp>
#include <uri_sensors/resources/laserscannergazebo.hpp>
#include <uri_uav/resources/iris_interface.hpp>
#include <uri_base/shared_memory.hpp>

#include <uri_bridge/resources/shared_memory.hpp>


#include <sensor_msgs/PointCloud.h>

#ifndef __URI_COLLECT3DSCAN_HPP__
#define __URI_COLLECT3DSCAN_HPP__



using namespace uri;
using namespace uri_base;


namespace uri_bridge{
	
	
	/// @brief This class is an example on how to implement a Task in uri.
	/// @details This class is derived from the class uri::Task. Any uri task must provide the method belonging to this class.
	/// uri users are encouraged to use this class as base to create nrew tasks. An automatic tool may be implemented in the future.
	class Collect3DScan: public Task{
		
		// ################ put here your private variables.
		
		// Resources
		uri_sensors::LaserScanner* ls;
		uri_uav_resources::IrisInterface* uav;
		
		Eigen::Vector3d _position;
		Eigen::Quaterniond _orientation;
		
		double first_yaw;
		double last_yaw;
		bool first_yaw_selected;
		double previous_yaw;
		double traveled_yaw;
		
		int counter;
		
		std::fstream fout;
		
		uri_bridge::PointCloud pc;
		
// 		uri_base::SharedMemory<uri_bridge::PointCloud>* pcl;
		uri_base::SharedMemory<uri_bridge::LaserScan>* lsc;
		
		uri_bridge::LaserScan new_2D_scan;
// 		std::vector<double> num_rays_per_scan;
// 		std::vector<Eigen::Vector2d> new_scan_polar;

		
		
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
		Collect3DScan();
		
		/// @brief get here the resources needed in the task from the ResourceVector.
		/// @details This method is mandatory since it is defined as purely virtual in the class uri::Task.
		/// @param[in] &res ResourceVector a vector containing the pointer to all the resources instantiated by the program.
		void get_mandatory_resources(ResourceVector &res);
		
	};
	
	PLUGINLIB_EXPORT_CLASS(uri_bridge::Collect3DScan, uri::Task)
	
	
};


#endif
