
#include <string>
#include <thread>

#include <pluginlib/class_list_macros.h>
#include <uri_core/resource.hpp>

// #include <Eigen/Geometry>


#ifndef __EXAMPLERESOURCE_HPP__
#define __EXAMPLERESOURCE_HPP__


using namespace uri;

namespace uri_example {
	
	
/// @brief An example of Resource.
/// @details This Resource does nothing. 
/// By default, a Resource contains an OptionVector _options and a ros::NodeHandle n.
/// Options are automatically initalized to the value in the configfile after the execution of the constructor.
class ExampleResource: public Resource{
		
		/// @brief Madatory initialization method.
		/// @details this method is called once after the instantiation is created. You can use it to initialize all your private variables
		void _init();
		
		// put here all your private variables
		
		
		// if you need a thread, you can use the c++11 standard 
		// std::thread* getcharacters;
		
	public:
		
		/// @brief Constructor
		/// @details This method does not build anything.
		ExampleResource();
		
};
} // end namespace uri_example

PLUGINLIB_EXPORT_CLASS(uri_example::ExampleResource, uri::Resource)




#endif






