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
		
		/// @brief Main constructor.
		/// @details Setup two Options: period with default value 0.1 and period_tollerance with default value 0.01. Puts some values at zero.
		BehaviorController();
		
		/// @brief Provides a pointer to the tist of behaviors
		void setBehaviorList(std::vector<uri::Behavior*> *_beh);
		
		/// @brief Provides a pointer to the tist of behaviors
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
		Behavior* behavior(std::string nm){
			
			for (int i=0; i < behaviors->size(); i++){
				if ( nm.compare(behaviors->at(i)->name())==0 ){
					return behaviors->at(i);
				}
			}
			
			return NULL;
		}


// 		
// 		/// @brief Returns the name of the task
// 		/// @return std::string containing the name of the task.
// 		std::string& name();
// 		
// 		/// @brief Mandatory initialization method.
// 		virtual void get_mandatory_resources(ResourceVector& res)=0;
// 		
// 		/// @brief This function is called at constant time period
// 		void run(const ros::TimerEvent&);
// 		
// 		/// @brief Returns the execution frequency of the task
// 		/// @return mean execution frequency [1/s]
// 		double frequency();
// 		
// 		/// @brief Activate the task - method run will be executed with constant time period options.period
// 		void activate_task();
// 		
// 		/// @brief Deactivate the task - method run will not be executed until new activation
// 		void deactivate_task();
// 		
// 		/// @brief Check wheter the task is active.
// 		/// @return \true if active, \false otherwise.
// 		bool task_active();
	};

	
	
};


#endif
