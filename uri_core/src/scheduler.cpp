#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <pluginlib/class_loader.h>

#include <tinyxml.h>

#include <uri_core/utilities.hpp>
#include <uri_core/resource.hpp>
#include <uri_core/task.hpp>
#include <uri_core/behavior.hpp>
#include <uri_core/scheduler.hpp>









namespace uri{


Scheduler::Scheduler(ros::NodeHandle &nh, std::string &config_file_name):n(nh){
	
	resource_loader = new pluginlib::ClassLoader<uri::Resource>("uri_core", "uri::Resource");
	
	task_loader = new pluginlib::ClassLoader<uri::Task>("uri_core", "uri::Task");
	
	behavior_controller_loader = new pluginlib::ClassLoader<uri::BehaviorController>("uri_core", "uri::BehaviorController");
	
	TiXmlDocument configuration_xml;
	if (!load_configuration_file(config_file_name.c_str(), configuration_xml)){
		ROS_FATAL("Terminating...");
	}
	
	load_resources(&configuration_xml);

	load_tasks(&configuration_xml);
	
	load_behaviors(&configuration_xml);
	
	behavior_controller_found = false;
	load_behavior_controller(&configuration_xml);
	if (not behavior_controller_found){
		ROS_FATAL("No behavior controller! Terminatiing...");
	}
	
	
	ROS_INFO("Activating " ANSI_COLOR_BEHAVIOR_CONTROLLER "%s" ANSI_COLOR_RESET ".", behavior_controller->name().c_str());
	behavior_controller->activate_task();
	
}






int Scheduler::dump_attribs_to_stdout(TiXmlElement* pElement)
{
	if ( !pElement ) return 0;

	TiXmlAttribute* pAttrib=pElement->FirstAttribute();
	int i=0;
	int ival;
	double dval;
	while (pAttrib)
	{
		printf( "   %s: value=[%s]", pAttrib->Name(), pAttrib->Value());
		if (pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS)    printf( " int=%d", ival);
		if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
		printf( "\n" );
		i++;
		pAttrib=pAttrib->Next();
	}
	return i;
}





// load the named file and dump its structure to STDOUT
bool Scheduler::load_configuration_file(const char* pFilename, TiXmlDocument &doc)
{
	ROS_INFO("Loading [%s].", pFilename);
	doc = TiXmlDocument(pFilename);
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		return true;
	}
	else
	{
		ROS_FATAL("Failed to load file [%s].", pFilename);
		return false;
	}
}





void Scheduler::load_resources( TiXmlNode* pParent)
{
	
// 	std::cout << "a.1" << std::endl;
	
	
	if ( !pParent ) return;
	TiXmlNode* pChild;
	int t = pParent->Type();
	int num;
	if ( t == TiXmlNode::TINYXML_ELEMENT )
	{
// 		printf( "Element [%s]\n", pParent->Value() );
		if(pParent->ValueStr().compare("resource") == 0){
			TiXmlAttribute* pAttrib=pParent->ToElement()->FirstAttribute();
			if (std::string(pAttrib->Name()).compare("name")!=0){
				ROS_FATAL("Malformed configfile! First element of a resource must be the resource name.");
			}
			else {
				ROS_INFO(ANSI_COLOR_RESOURCE "Creating resource [%s]." ANSI_COLOR_RESET, pAttrib->Value());
				try
				{
					boost::shared_ptr<uri::Resource> newresource = resource_loader->createInstance(pAttrib->Value());
					pAttrib = pAttrib->Next();
					newresource->init(n, pAttrib);
					resources.push_back(newresource);
				}
				catch(pluginlib::PluginlibException& ex)
				{
					ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
				}
			}
// 			num=dump_attribs_to_stdout(pParent->ToElement());
		}
	}
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		load_resources( pChild);
	}
// 	std::cout << "a.2" << std::endl;
}







void Scheduler::load_tasks( TiXmlNode* pParent)
{
	if ( !pParent ) return;
	TiXmlNode* pChild;
	int t = pParent->Type();
	int num;
	if ( t == TiXmlNode::TINYXML_ELEMENT )
	{
// 		printf( "Element [%s]\n", pParent->Value() );
		if(pParent->ValueStr().compare("task") == 0){
			TiXmlAttribute* pAttrib=pParent->ToElement()->FirstAttribute();
			if (std::string(pAttrib->Name()).compare("name")!=0){
				ROS_FATAL("Malformed configfile! First element of a task must be the task name.");
			}
			else {
				ROS_INFO(ANSI_COLOR_TASK "Creating task [%s]." ANSI_COLOR_RESET, pAttrib->Value());
				try
				{
					boost::shared_ptr<uri::Task> newtask = task_loader->createInstance(pAttrib->Value());
					pAttrib = pAttrib->Next();
					newtask->init(n, pAttrib);
					newtask->get_mandatory_resources(resources);
					tasks.push_back(newtask);
				}
				catch(pluginlib::PluginlibException& ex)
				{
					ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
				}
			}
// 			num=dump_attribs_to_stdout(pParent->ToElement());
		}
	}
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		load_tasks( pChild);
	}
}






void Scheduler::load_behavior_controller( TiXmlNode* pParent)
{
	if ( !pParent ) return;
	TiXmlNode* pChild;
	int t = pParent->Type();
	int num;
	if ( t == TiXmlNode::TINYXML_ELEMENT )
	{
// 		printf( "Element [%s]\n", pParent->Value() );
		if(pParent->ValueStr().compare("behavior_controller") == 0){
			if (behavior_controller_found){
				ROS_FATAL("Malformed configfile! Two or more behavior controllers found!! Terminating...");
			}
			TiXmlAttribute* pAttrib=pParent->ToElement()->FirstAttribute();
			if (std::string(pAttrib->Name()).compare("name")!=0){
				ROS_FATAL("Malformed configfile! First element of a behavior controller must be the task name.");
			}
			else {
				ROS_INFO(ANSI_COLOR_BEHAVIOR_CONTROLLER "Creating behavior controller [%s]." ANSI_COLOR_RESET, pAttrib->Value());
				try
				{
					behavior_controller = behavior_controller_loader->createInstance(pAttrib->Value());
					pAttrib = pAttrib->Next();
					behavior_controller->init(n, pAttrib);
					behavior_controller->get_mandatory_resources(resources);
					behavior_controller->setBehaviorList(&behaviors);
					behavior_controller->setTaskList(&tasks);
					behavior_controller_found = true;
				}
				catch(pluginlib::PluginlibException& ex)
				{
					ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
				}
			}
// 			num=dump_attribs_to_stdout(pParent->ToElement());
		}
	}
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		load_behavior_controller(pChild);
	}
}











void Scheduler::load_behaviors( TiXmlNode* pParent)
{
// 		std::cout << "c.1" << std::endl;

	if ( !pParent ) return;
	TiXmlNode* pChild;
	int t = pParent->Type();
// 	int num;
	if ( t == TiXmlNode::TINYXML_ELEMENT )
	{
// 		std::cout << "c.2" << std::endl;
// 		printf( "Element [%s]\n", pParent->Value() );
		if(pParent->ValueStr().compare("behavior") == 0){
			TiXmlAttribute* pAttrib=pParent->ToElement()->FirstAttribute();
			if (std::string(pAttrib->Name()).compare("name")!=0){
				ROS_FATAL("Malformed configfile! First attribute of a behavior must be the behavior name.");
			}
			else {
// 		std::cout << "c.3" << std::endl;
				std::string behavior_name(pAttrib->Value());
				ROS_INFO(ANSI_COLOR_BEHAVIOR "Creating behavior [%s]." ANSI_COLOR_RESET, behavior_name.c_str());
				pAttrib=pAttrib->Next();
				if (std::string(pAttrib->Name()).compare("tasklist")!=0){
					ROS_FATAL("Malformed configfile! Second attribute of a behavior must be the task list.");
				}
				else {
// 		std::cout << "c.4" << std::endl;
// 					num=dump_attribs_to_stdout(pParent->ToElement());
					uri::Behavior* newbeh = new uri::Behavior(behavior_name, tasks, pAttrib->ValueStr());
					behaviors.push_back(newbeh);
				}
			}
		}
	}
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
// 		std::cout << "c.5" << std::endl;
		load_behaviors( pChild);
	}
}





// 	void Scheduler::run(const ros::TimerEvent&){
// 		
// 		_callbacks++;
// 		
// 		// in this case somebody has requested a behavior change (either a task or the operator)
// 		if (behavior_controller->activeBehavior() != behavior_controller->nextActiveBehavior()){
// // 			std::cout  << "d.2" << std::endl;			
// 			
// 			if (behavior_controller->activeBehavior() == NULL){
// 				
// 				ROS_INFO("Activating "ANSI_COLOR_BEHAVIOR_CONTROLLER"%s"ANSI_COLOR_RESET".", behavior_controller->name().c_str());
// 				behavior_controller->activate_task();
// 				
// 				ROS_INFO("Setting first behavior to "ANSI_COLOR_BEHAVIOR"%s"ANSI_COLOR_RESET".", behavior_controller->nextActiveBehavior()->name().c_str());
// 				
// 				for(int i=0; i<tasks.size(); i++){
// 					if (behavior_controller->nextActiveBehavior()->contains_task(tasks[i]->name())>=0){
// 						tasks[i]->activate_task();
// 						ROS_INFO("  Activating "ANSI_COLOR_TASK"%s"ANSI_COLOR_RESET".", tasks[i]->name().c_str());
// 					}
// 					else{}
// 				}
// 			}
// 			else {
// 				
// 				ROS_INFO("Switching from behavior "ANSI_COLOR_BEHAVIOR"%s"ANSI_COLOR_RESET" to behavior "ANSI_COLOR_BEHAVIOR"%s"ANSI_COLOR_RESET".", behavior_controller->activeBehavior()->name().c_str(), behavior_controller->nextActiveBehavior()->name().c_str());
// // 				_active_behavior->print();
// // 				behavior_controller->nextActiveBehavior()->print();
// 				
// 				for(int i=0; i<tasks.size(); i++){
// 					
// // 					std::cout << "   " << tasks[i]->name()  << " " << _active_behavior->contains_task(tasks[i]->name()) << " " << _next_active_behavior->contains_task(tasks[i]->name()) << std::endl;
// 					
// 					if (behavior_controller->activeBehavior()->contains_task(tasks[i]->name())>=0 && behavior_controller->nextActiveBehavior()->contains_task(tasks[i]->name())<0){
// 						tasks[i]->deactivate_task();
// 						ROS_INFO("  Deactivating "ANSI_COLOR_TASK"%s"ANSI_COLOR_RESET".", tasks[i]->name().c_str());
// 					}
// 					else if (behavior_controller->activeBehavior()->contains_task(tasks[i]->name())<0 && behavior_controller->nextActiveBehavior()->contains_task(tasks[i]->name())>=0){
// 						tasks[i]->activate_task();
// 						ROS_INFO("  Activating "ANSI_COLOR_TASK"%s"ANSI_COLOR_RESET".", tasks[i]->name().c_str());
// 					}
// 					else {}
// 				}
// 			}
// 			behavior_controller->nextActiveBehaviorImplemented();
// 		}
// 		
// 		
// // 				std::cout  << "d.5" << std::endl;			
// 		
// 		
// 		for(int i=0; i<behavior_controller->activeBehavior()->num_tasks(); i++){
// 			if (_callbacks%5000 == 0){
// 				ROS_INFO("  Frequency of %s is %f", behavior_controller->activeBehavior()->task(i)->name().c_str(), behavior_controller->activeBehavior()->task(i)->frequency());
// 			}
// 		}
// 		
// /*		
// 			if (_callbacks%7000 == 0){
// 				_next_active_behavior = behaviors[1];
// // 				std::cout  << "freq " <<  _active_behavior->task(i)->name() << " " << _active_behavior->task(i)->frequency() << std::endl;			
// 			}
// 		
// 			if (_callbacks%17000 == 0){
// 				_next_active_behavior = behaviors[2];
// // 				std::cout  << "freq " <<  _active_behavior->task(i)->name() << " " << _active_behavior->task(i)->frequency() << std::endl;			
// 			}
// 		
// 			if (_callbacks%23000 == 0){
// 				_next_active_behavior = behaviors[0];
// // 				std::cout  << "freq " <<  _active_behavior->task(i)->name() << " " << _active_behavior->task(i)->frequency() << std::endl;			
// 			}*/
// 		
// /*		
// 		for(int i=0; i<behaviors[_active_behavior]->num_tasks(); i++){
// // 			std::cout << behaviors.size() << " " << i << " " << behaviors[0]->task(i)->time_to_next_execution() << std::endl;
// 			if (!behaviors[_active_behavior]->task(i)->task_active()){
// // 				std::cout << behaviors.size() << " " << i << " " << behaviors[0]->task(i)->time_to_next_execution() << std::endl;
// // 				behaviors[_active_behavior]->task(i)->run();
// // 				std::cout  << "freq " <<  behaviors[_active_behavior]->task(i)->name() << " " << behaviors[_active_behavior]->task(i)->frequency() << std::endl;
// 				behaviors[_active_behavior]->task(i)->activate_task();
// 			}
// 			if (_callbacks%1000 == 0){
// 				std::cout  << "freq " <<  behaviors[_active_behavior]->task(i)->name() << " " << behaviors[_active_behavior]->task(i)->frequency() << std::endl;			
// 			}
// 		}*/
// 		
// 		
// 		
// 	}
// 
// 
// 
};




















