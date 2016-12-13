#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <tinyxml.h>

#include <uri_core/utilities.hpp>
#include <uri_core/element.hpp>
#include <uri_core/resource.hpp>


#ifndef __URI_TASK_HPP__
#define __URI_TASK_HPP__



namespace uri{
	
	
	
	enum TaskOutput{
		Continue,
		Terminate
	};
	
	
	
/// @brief Base class for any task to be executed in uri.
/// @details This class is the base class for any task that can run in URI.
	class Task: public Element{
		
	private:
		
		/// @brief termination condition of the last call of run().
		TaskOutput _output;
		
		/// @brief Estimated mean execution time.
		double _meanExTime;
		
		/// @brief Number of executions since last activation of the task.
		double _executions;
		
		/// @brief Time of first execution since last activation.
		double _first_execution_time;
		
		/// @brief Set \b true at each first execution after the last activation. Set \b false anytime the task is deactivated.
		bool _first_execution;
		
		/// @brief ros::Timer that temporizes the execution of the run function.
		ros::Timer _timer;
		
		/// @brief \b true is the task active, \b false otherwise
		bool _task_active;
		
		/// @brief Mandatory initialization method.
		void _init();
		
		/// @brief This method is executed anytime a Task is activated through the method activate_task();
		/// @details This method is purely virtual within the Task class, and must be implemented by any derived Task.
		virtual void _initialize()=0;
		
		/// @brief This method is executed anytime a Task is activated through the method activate_task();
		/// @details This method is purely virtual within the Task class, and must be implemented by any derived Task.
		virtual void _activate()=0;
		
		/// @brief This method is executed anytime a Task is deactivated through the method deactivate_task();
		/// @details This method is purely virtual within the Task class, and must be implemented by any derived Task.
		virtual void _deactivate()=0;
		
		/// @brief This method contains the standard routine and is called anytime the _timer executes the callback.
		/// @details This method is purely virtual within the Task class, and must be implemented by any derived Task.
		virtual TaskOutput _run()=0;
		
		
	public:
		
		/// @brief Main constructor.
		/// @details Setup two Options: period with default value 0.1 and period_tollerance with default value 0.01. Puts some values at zero.
		Task();
		
		/// @brief Returns the name of the task
		/// @return std::string containing the name of the task.
		std::string& name();
		
		/// @brief This function is called at constant time period
		void run(const ros::TimerEvent&);
		
		/// @brief Returns the execution frequency of the task
		/// @return mean execution frequency [1/s]
		double frequency();
		
		/// @brief Activate the task - method run will be executed with constant time period options.period
		void activate_task();
		
		/// @brief Deactivate the task - method run will not be executed until new activation
		void deactivate_task();
		
		/// @brief Check wheter the task is active.
		/// @return \true if active, \false otherwise.
		bool task_active();
		
		/// @brief Returns the number of executions of the task since last activation (0 if task is not active).
		/// @return Number of executions since last activation.
		int executions();
		
		/// @brief Inquiry if the task requested a termination.
		/// @return Returns \b true if the last executions returned a Terminate statement, \b false otherwise.
		bool terminate();
		
		/// @brief Find the resources required in the task.
		/// @details This method is purely virtual within the Task class, and must be implemented by any derived Task.
		virtual void get_mandatory_resources(ResourceVector& res)=0;
		
		
	};

	
	
};


#endif
