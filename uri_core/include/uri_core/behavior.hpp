#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>
#include <uri_core/task.hpp>

#ifndef __URI_BEHAVOUR_HPP__
#define __URI_BEHAVOUR_HPP__

namespace uri{
	
	class Behavior{
		
		std::string _name;
		
		int _id;
		
		std::vector<boost::shared_ptr<uri::Task> > _task;
		
	public:
		
		/// standard constructor
		Behavior();
		
		/// name constructor
		Behavior(std::string &name, std::vector<boost::shared_ptr<uri::Task> > &tasks, const std::string &required_tasks);
		
		
		boost::shared_ptr<uri::Task> task(int i){
			return _task[i];
		}
		
		int num_tasks(){
			return _task.size();
		}
		
		std::string& name(){
			return _name;
		}
		
		int contains_task(std::string &tn){
			
			for (int i=0; i<_task.size(); i++){
				if (tn.compare(_task[i]->name())==0){
					return i;
				}
			}
			return -1;
		}
		
		void print(){
			std::cout << "Behavior: " << _name << std::endl;
			std::cout << " # of tasks: " << _task.size() << std::endl;
			for (int i=0; i < _task.size(); i++){
				std::cout << _task[i]->name() << std::endl;
			}
		}
		
		bool terminate(){
			for (int i=0; i < _task.size(); i++){
				if (_task[i]->terminate()) {
					return true;
				}
			}
			return false;
		}
		
	};
	
};

#endif

