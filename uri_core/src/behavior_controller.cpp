#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/behavior_controller.hpp>

#include <std_msgs/Header.h>


namespace uri{

  

BehaviorController::BehaviorController():Task(){
	
	_active_behavior = NULL;
	_next_active_behavior = NULL;
	
// 	_options.addDoubleOption("period",0.1);
// 	_options.addDoubleOption("period_tollerance",0.01);
// 	_meanExTime = 0.0;
// 	_executions = 0.0;
// 	_first_execution = false;
// 	_task_reports_subscriber = n->subscribe("task_report", 1, &BehaviorController::_task_reports_CB, this);
}

// void BehaviorController::_task_reports_CB(const std_msgs::Header::ConstPtr& msg){
// 	
// 	for(int i=0; i< tasks->size(); i++){
// 		if (tasks->at(i)->name().compare(msg->frame_id) == 0){
// 			tasks_reports[i] = (TaskOutput)msg->seq;
// 			break;
// 		}
// 	}
// 	
// // 	behavior(msg->frame_id);
// }


void BehaviorController::setBehaviorList(std::vector<uri::Behavior*> *_beh){
	behaviors = _beh;
// 	_active_behavior = NULL;
}


void BehaviorController::setTaskList(std::vector<boost::shared_ptr<uri::Task> > *_ta){
	tasks = _ta;
	
	for(int i=0; i< tasks->size(); i++){
		tasks_reports.push_back(Continue);
	}
}



Behavior* BehaviorController::behavior(std::string nm){
	
	for (int i=0; i < behaviors->size(); i++){
		if ( nm.compare(behaviors->at(i)->name())==0 ){
			return behaviors->at(i);
		}
	}
	
	return NULL;
}




TaskOutput BehaviorController::_run(){
	
	if (_active_behavior == _next_active_behavior){
		__run();
	}
	
	// in this case somebody has requested a behavior change (either a task or the operator)
	if (_active_behavior != _next_active_behavior){
			
		if (_active_behavior == NULL){
				
			ROS_INFO("Setting first behavior to " ANSI_COLOR_BEHAVIOR "%s" ANSI_COLOR_RESET ".", _next_active_behavior->name().c_str());
				
			for(int i=0; i<tasks->size(); i++){
				if (_next_active_behavior->contains_task(tasks->at(i)->name())>=0){
					ROS_INFO("  Activating " ANSI_COLOR_TASK "%s" ANSI_COLOR_RESET ".", tasks->at(i)->name().c_str());
					tasks->at(i)->activate_task();
				}
				else{}
			}
		}
		else {
				
			ROS_INFO("Switching from behavior " ANSI_COLOR_BEHAVIOR "%s" ANSI_COLOR_RESET " to behavior " ANSI_COLOR_BEHAVIOR "%s" ANSI_COLOR_RESET ".", _active_behavior->name().c_str(), _next_active_behavior->name().c_str());
// 				_active_behavior->print();
// 				_next_active_behavior->print();
			
			for(int i=0; i<tasks->size(); i++){
				
// 					std::cout << "   " << tasks[i]->name()  << " " << _active_behavior->contains_task(tasks[i]->name()) << " " << _next_active_behavior->contains_task(tasks[i]->name()) << std::endl;
				
				if (_active_behavior->contains_task(tasks->at(i)->name())>=0 && _next_active_behavior->contains_task(tasks->at(i)->name())<0){
					ROS_INFO("  Deactivating " ANSI_COLOR_TASK "%s" ANSI_COLOR_RESET ".", tasks->at(i)->name().c_str());
					tasks->at(i)->deactivate_task();
				}
				else if (_active_behavior->contains_task(tasks->at(i)->name())<0 && _next_active_behavior->contains_task(tasks->at(i)->name())>=0){
					ROS_INFO("  Activating " ANSI_COLOR_TASK "%s" ANSI_COLOR_RESET ".", tasks->at(i)->name().c_str());
					tasks->at(i)->activate_task();
				}
				else if (_active_behavior->contains_task(tasks->at(i)->name())>=0 && _next_active_behavior->contains_task(tasks->at(i)->name())>=0){
					ROS_INFO("  Resetting " ANSI_COLOR_TASK "%s" ANSI_COLOR_RESET ".", tasks->at(i)->name().c_str());
					tasks->at(i)->reset_task();
				}
				else {}
			}
		}
		this->nextActiveBehaviorImplemented();
		_active_behavior->increase_counter();
	}
	
	for(int i=0; i<_active_behavior->num_tasks(); i++){
		if (executions()%5000 == 0){
			ROS_INFO("  Frequency of %s is %f", _active_behavior->task(i)->name().c_str(), _active_behavior->task(i)->frequency());
		}
	}
};



};


