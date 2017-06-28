#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <tinyxml.h>

#include <uri_core/utilities.hpp>
#include <uri_core/element.hpp>
#include <uri_core/resource.hpp>
#include <uri_core/task.hpp>
#include <uri_core/behavior.hpp>

#include <std_msgs/Header.h>


#ifndef __URI_BEHAVIOR_CONTROLLER_HPP__
#define __URI_BEHAVIOR_CONTROLLER_HPP__



// green
#define ANSI_COLOR_OPTION   "\x1b[95m"
// green
#define ANSI_COLOR_RESOURCE   "\x1b[32m"
// yellow
#define ANSI_COLOR_BEHAVIOR  "\x1b[33m"
// blue
#define ANSI_COLOR_TASK    "\x1b[34m"
// cyan
#define ANSI_COLOR_BEHAVIOR_CONTROLLER    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

namespace uri{
	
	
	
/// @brief Base class for any task to be executed in uri.
/// @details This class is the base class for any task that can run in URI.
	class BehaviorController: public Task{
		
	private:
		
// 		ros::Subscriber _task_reports_subscriber;
// 		void _task_reports_CB(const std_msgs::Header::ConstPtr& msg);
		
		/// @brief virtual function. Any derived task must provide this function. It is called anytime the _timer executes the callback.
		virtual TaskOutput __run()=0;
		
		/// @brief virtual function. Any derived task must provide this function. It is called anytime the _timer executes the callback.
		TaskOutput _run();
		
	protected:
		
		std::vector<boost::shared_ptr<uri::Task> > *tasks;
		std::vector< TaskOutput > tasks_reports;
		
		std::vector<uri::Behavior*> *behaviors;
		
		uri::Behavior* _active_behavior;
		
		uri::Behavior* _next_active_behavior;
		
	public:
		
		OptionVector global_options;
		
		/// @brief Main constructor.
		/// @details Setup two Options: period with default value 0.1 and period_tollerance with default value 0.01. Puts some values at zero.
		BehaviorController();
		
		/// @brief Provides a pointer to the list of behaviors
		void setBehaviorList(std::vector<uri::Behavior*> *_beh);
		
		/// @brief Provides a pointer to the list of tasks
		void setTaskList(std::vector<boost::shared_ptr<uri::Task> > *_ta);

		
		/// @brief Returns a pointer to the active behavior
		uri::Behavior* activeBehavior(){
			return _active_behavior;
		}
		
		/// @brief Returns a pointer to the next active behavior
		uri::Behavior* nextActiveBehavior(){
			return _next_active_behavior;
		}
		
		/// @brief to be used once the behaviors have been swapped.
		void nextActiveBehaviorImplemented(){
			_active_behavior = _next_active_behavior;
		}
		
		/// @brief finds a behavior with the specified name.
		/// @param[in] nm std::string containing the name of the behavior.
		/// @return A pointer to the specified behavior if it exists. A NULL pointer of the specified behavior does not exists
		Behavior* behavior(std::string nm);
		

		void set_init_time(){
			if (not init_time_saved){
				init_time = ros::Time::now().toSec();
				init_time_saved = true;
			}
		}
	};

	
	
};


#endif
