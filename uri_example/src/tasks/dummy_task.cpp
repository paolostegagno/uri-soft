#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_example/tasks/dummy_task.hpp>

namespace uri_example{

  
  
  

DummyTask::DummyTask():Task()/*:_name(nm)*/{
	_name = "uri::DummyTask";
	_options.addDoubleOption("pp",10);
}


TaskOutput DummyTask::_run(){
// 	std::cout << "DummyTask _run!!!!!!!!!!!!!" << std::endl;
// 	std::cout << "Begin " << this->name() << std::endl;
// 	for (int i =0; i<1000; i++){
		usleep(1000);
// 	}
// 	std::cout << "End   " << this->name() << std::endl;
	
}

void DummyTask::get_mandatory_resources(ResourceVector &res){
	
}


// int DummyTask::_set_options(TiXmlAttribute* attribute){
// 	ROS_INFO("WARNING");
// }




};


