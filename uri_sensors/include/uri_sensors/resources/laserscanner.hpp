
#include <string>
#include <thread>

#include <pluginlib/class_list_macros.h>
#include <uri_core/resource.hpp>

// #include <Eigen/Geometry>


#ifndef __LASERSCANNER_HPP__
#define __LASERSCANNER_HPP__


using namespace uri;

namespace uri_sensors {
	
	
/// @brief An example of Resource.
/// @details This Resource does nothing. 
/// By default, a Resource contains an OptionVector _options and a ros::NodeHandle n.
/// Options are automatically initalized to the value in the configfile after the execution of the constructor.
class LaserScanner: public Resource{
		
		/// @brief Madatory initialization method.
		/// @details this method is called once after the instantiation is created. You can use it to initialize all your private variables
		void _init();
		
		// put here all your private variables
		
		
		// if you need a thread, you can use the c++11 standard 
		// std::thread* getcharacters;
		
	public:
		
		/// @brief Constructor
		/// @details This method does not build anything.
		LaserScanner();
		
};
} // end namespace uri_sensors

PLUGINLIB_EXPORT_CLASS(uri_sensors::LaserScanner, uri::Resource)




#endif






