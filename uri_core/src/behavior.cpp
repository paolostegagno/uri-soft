#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <uri_core/utilities.hpp>
#include <uri_core/behavior.hpp>


namespace uri{
	
		
	Behavior::Behavior(){
		
	}
	
	
	Behavior::Behavior(std::string &name, std::vector<boost::shared_ptr<uri::Task> > &tasks, const std::string &required_tasks){
		_name = name;
		_task.clear();
		
		std::string required_task;
		std::stringstream tasklist(required_tasks);
		while (!tasklist.eof()) {
			tasklist >> required_task;
			bool found = false;
			for (int j=0; j<tasks.size(); j++){
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
		_counter = 0;
	}
	
	int Behavior::contains_task(std::string &tn){
		
		for (int i=0; i<_task.size(); i++){
			if (tn.compare(_task[i]->name())==0){
				return i;
			}
		}
		return -1;
	}
	
	
	
	bool Behavior::set_option_double(std::string &oname, std::string &tname, double value){
		int tnumber = this->contains_task(tname);
		if (tnumber<0){
			ROS_ERROR("  Can't find task %s in Behavior %s.", tname.c_str(), _name.c_str());
			return false;
		}
		return _task[tnumber]->set_option_double(oname, value);
	}
	
	bool Behavior::set_option_double(const char* oname_c, const char* tname_c, double value){
		
		std::string oname(oname_c);
		std::string tname(tname_c);
		
		int tnumber = this->contains_task(tname);
		if (tnumber<0){
			ROS_ERROR("  Can't find task %s in Behavior %s.", tname.c_str(), _name.c_str());
			return false;
		}
		return _task[tnumber]->set_option_double(oname, value);
	}

	
	
	bool Behavior::set_option_bool(std::string &oname, std::string &tname, bool value){
		int tnumber = this->contains_task(tname);
		if (tnumber<0){
			ROS_ERROR("  Can't find task %s in Behavior %s.", tname.c_str(), _name.c_str());
			return false;
		}
		return _task[tnumber]->set_option_bool(oname, value);
	}
	
	bool Behavior::set_option_string(std::string &oname, std::string &tname, std::string value){
		int tnumber = this->contains_task(tname);
		if (tnumber<0){
			ROS_ERROR("  Can't find task %s in Behavior %s.", tname.c_str(), _name.c_str());
			return false;
		}
		return _task[tnumber]->set_option_string(oname, value);
	}
	
	bool Behavior::set_option_int(std::string &oname, std::string &tname, int value){
		int tnumber = this->contains_task(tname);
		if (tnumber<0){
			ROS_ERROR("  Can't find task %s in Behavior %s.", tname.c_str(), _name.c_str());
			return false;
		}
		return _task[tnumber]->set_option_int(oname, value);
	}
	
	
	
	
	
	
	
	void Behavior::print(){
		std::cout << "Behavior: " << _name << std::endl;
		std::cout << " # of tasks: " << _task.size() << std::endl;
		for (int i=0; i < _task.size(); i++){
			std::cout << _task[i]->name() << std::endl;
		}
	}
	
	bool Behavior::terminate(){
		for (int i=0; i < _task.size(); i++){
			if (_task[i]->terminate()) {
				return true;
			}
		}
		return false;
	}


	
};



