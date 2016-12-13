#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/task.hpp>

#include <std_msgs/Header.h>


namespace uri{

  

Task::Task(){
	
	_options.addDoubleOption("period",0.1);
	_options.addDoubleOption("period_tollerance",0.01);
	_meanExTime = 0.0;
	_executions = 0.0;
	_first_execution = false;
	_output = Continue;
}

std::string& Task::name(){
	return _name;
}



void Task::run(const ros::TimerEvent&){
	
	
	if (!_first_execution){
		_first_execution=true;
		_first_execution_time = ros::Time::now().toSec();
	}
	
	ros::Time begin = ros::Time::now();
	
	_output = _run();
	
	double thisExTime = (ros::Time::now() -begin).toSec();
	_executions++;
	_meanExTime = ((_executions-1.0)*_meanExTime + thisExTime)/_executions;
	
}


void Task::_init(){

	_task_active = false;
	// and use it to create the run Timer
	_timer = n->createTimer(ros::Duration(_options["period"]->getDoubleValue()), &Task::run, this, false, false);
	
	_initialize();
// 	_reporter = n->advertise<std_msgs::Header>("task_report", 100);

}



double Task::frequency(){
	return _executions/(ros::Time::now().toSec() - _first_execution_time); 
}

void Task::activate_task(){
	_timer.start();
	_task_active = true;
	_output = Continue;
	_activate();
}

void Task::deactivate_task(){
	_timer.stop();
	_task_active = false;
	_first_execution=false;
	_output = Continue;
	_executions=0;
	_deactivate();
}


bool Task::task_active(){
	return _task_active;
}

int Task::executions(){
	return _executions;
}

bool Task::terminate(){
	if (_output == Terminate){
		return true;
	}
	else if (_output == Continue) {
		return false;
	}
	else return false;
}

};


