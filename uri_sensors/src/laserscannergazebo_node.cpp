

#include "ros/ros.h"

#include <sstream>
#include <iostream>
#include <string>


#include "uri_sensors/resources/laserscannergazebo.hpp"



#include <pluginlib/class_loader.h>


using namespace uri_sensors;




int main(int argc, char **argv)
{
	ros::init(argc, argv, "urimav");
	ros::NodeHandle n;
	
	
	LaserScannerGazebo ls(n);
	
	pluginlib::ClassLoader<mip2::Resource> *resource_loader;
	resource_loader = new pluginlib::ClassLoader<mip2::Resource>("mip2_core", "uri_uav::Resource");
	
	boost::shared_ptr<mip2::Resource> newresource = resource_loader->createInstance("uri_uav_resources::IrisInterface");

	
	IrisInterface* uav_interface = (IrisInterface*)newresource->ptr();
//   IrisInterface uav_interface;
	TiXmlAttribute* attr=NULL;
  uav_interface->init(n, attr);
  
  uav_interface->setMode("guided");
	
	
	
	
  
  while (!uav_interface->local_position_pose_received() && ros::ok()){
    usleep(100000);
    ros::spinOnce();
  }
  
  uav_interface->armThrottle();
  
  usleep(100000);
  
  uav_interface->takeoff(30);
  
  sleep(10);
  
  for (int i=0; i<1000; i++){
    std::cout << "!!!!!!!!!!!!!!!!!" << std::endl;
    uav_interface->commandVelocity(1.0, 1.0, 1.0);
    usleep(100000);
    if (!ros::ok()) break;
  }
  
  ros::spin();
  
  return 0;
}



