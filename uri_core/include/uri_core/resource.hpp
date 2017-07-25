#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <uri_core/utilities.hpp>
#include <uri_core/element.hpp>

#ifndef __URI_RESOURCE_HPP__
#define __URI_RESOURCE_HPP__

/// @brief this namespace contains the base classes for the execution of uri-soft
namespace uri{
	
	
	
	/// @brief an available cluster of data and functionalities
	class Resource:public Element{
			
		private:
			
		protected:
			/// @brief Name of the element.
			std::string _type;
			
		public:
			/// @brief default contructor (does nothing).
			Resource();
			
			/// @brief Contructor sets the NodeHandle.
			Resource(ros::NodeHandle &_n);
			
			/// @brief pointer to this Resource.
			Resource* ptr();
			
			/// @brief Returns the type of the element
			/// @return std::string containing the type of the element.
			std::string& type();

	};
	
	
	
	/// @brief A std::vector of boost::shared_ptr<Resources>
	class ResourceVector : public std::vector<boost::shared_ptr<uri::Resource> > {
		
		public:
			/// @brief finds a Resource in the vector given a name.
			/// @details this methods search in the Resource vector a Resource named nm
			/// @param[in] nm the name of the Resource you are looking for
			/// @return if a Resource with name nm exists, a pointer to it, otherwise it returns NULL 
			Resource* get_resource_ptr(std::string nm, bool mandatory=true);
			
			/// @brief finds a Resource in the vector given a name.
			/// @details this methods search in the Resource vector a Resource named nm
			/// @param[in] nm the name of the Resource you are looking for
			/// @param[in] nam the name of the Resource you are looking for
			/// @return if a Resource with name nm exists, a pointer to it, otherwise it returns NULL 
			Resource* get_resource_ptr(std::string nm, std::string nam);

			
	};

	
	
};



#endif

