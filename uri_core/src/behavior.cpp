#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <uri_core/utilities.hpp>
#include <uri_core/behavior.hpp>


namespace uri{
	
		
		/// standard constructor
		Behavior::Behavior(){
			
		}
		
		
		/// name constructor
		Behavior::Behavior(std::string &name, std::vector<boost::shared_ptr<uri::Task> > &tasks, const std::string &required_tasks){
			_name = name;
			_task.clear();
			
			std::string required_task;
			std::stringstream tasklist(required_tasks);
			while (!tasklist.eof()) {
				tasklist >> required_task;
// 				std::cout << required_task << std::endl;
				bool found = false;
				for (int j=0; j<tasks.size(); j++){
// 					std::cout << tasks[j]->name() << std::endl;
					if (tasks[j]->name().compare(required_task)==0){
						_task.push_back(tasks[j]);
						ROS_INFO("  Found task %s required by behavior %s.", required_task.c_str(), _name.c_str());
						found = true;
					}
				}
				if (!found){
					ROS_ERROR("  Can't find task %s required by behavior %s.", required_task.c_str(), _name.c_str());
				}
			}
// 			for (int i=0; i<requiredTask.size(); i++){
// 			}
			
		}
		
		
	


	
};



