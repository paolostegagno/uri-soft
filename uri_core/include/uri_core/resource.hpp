#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <uri_core/utilities.hpp>
#include <uri_core/element.hpp>

#ifndef __URI_RESOURCE_HPP__
#define __URI_RESOURCE_HPP__


namespace uri{
	
	class Resource:public Element{
			
		private:

			
		public:
			Resource();
			
			Resource(ros::NodeHandle &_n);
			
			Resource* ptr();
		
	};
	
	
	
	class ResourceVector : public std::vector<boost::shared_ptr<uri::Resource> > {
		
		public:
			Resource* get_resource_ptr(std::string nm);
		
	};

	
	
};



#endif

