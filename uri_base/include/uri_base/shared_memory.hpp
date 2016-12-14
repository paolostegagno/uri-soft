

#include <string>



#include <pluginlib/class_list_macros.h>
#include <uri_core/resource.hpp>

#include <Eigen/Geometry>




#ifndef __SHARED_MEMORY_HPP__
#define __SHARED_MEMORY_HPP__

using namespace uri;

namespace uri_base {
	
	
/// @brief A piece of memory of type TYPE shared by multiple Tasks.
/// @details This template contains a piece of memory shared by multiple tasks.
/// This piece of memory is only accessible through get/set operators, and is mutexed, i.e.: two tasks cannot get/set it simultaneously.
/// To use this template class, you have to:
/// 1) Create your own class or data struct that you want to use to exchange data between two or more tasks.
///    \em tip: make sure that your class has the following mandatory methods:
///    null constructor of the type: YourClassName(){do something}
///    name method of the type: std::string name(){return "YourNamespace::YourClassName";}
/// 2) add a line at the end of your file in the format
///  PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<YourNamespace::YourClassName>, uri_base::Resource)
/// 3) create a plugin file in your package and insert an entry class mip2::SharedMemory<YourNamespace::YourClassName> as
/// <class type="uri_base::SharedMemory<YourNamespace::YourClassName>" base_class_type="uri_base::Resource">
/// <description> DESCRIPTION OF THE SHARED MEMORY. </description>
/// </class>
/// 4) modify your package.xml file to export the plugin file that you have created.
///
/// For an example on how to create a uri_base::SharedMemory object, see:
/// - trajectory.hpp file in this folder;
/// - package.xml file in this package;
/// - uri_base_plugin.xml file in this package.
template<class TYPE> class SharedMemory: public Resource{
		
		/// @brief Madatory initialization method.
		/// @details this method is called once after the instantiation is created.
		void _init(){
		}
		
		/// @brief This contains the actual data.
		TYPE _data;
		
		/// @brief This @b bool acts as a mutex for the data.
		bool _busy;
		
		/// @brief This @b bool inidicates whether the data has ever been set.
		bool _set;
		
		/// @brief This time is the value of the request.
		ros::Time _time_request;
		
	public:
		
		/// @brief Do-nothing constructor
		/// @details This method does not build anything. To be followed by initialize(ros::NodeHandle &n).
		SharedMemory() : _data() {
			std::stringstream namess;
			namess << "uri_base::SharedMemory<" << _data.name() << ">";
			_name = namess.str();
			_busy = false;
			_set = false;
		};
		
		/// @brief Gets the data stored in the shared memory.
		/// @param[out] out If return is \b true, the data stored in the shared memory are copied here.
		/// @param[in] timeout_s Maximum time limit to get the data.
		/// @return \b true if the data stored in the shared memory are available to be copied within the time limit, \b false otherwise.
		bool get(TYPE &out, double timeout_s){
			_time_request = ros::Time::now();
			while(_busy){
				if ((ros::Time::now()-_time_request).toSec()>timeout_s){
					return false;
				}
			}
			_busy = true;
			out = _data;
			_busy = false;
			return true;
		}
		
		/// @brief Sets the data stored in the shared memory.
		/// @param[in] in If return is \b true, the data stored in this variable are copied in the shared memory.
		/// @param[in] timeout_s Maximum time limit to set the data.
		/// @return \b true if the data stored in the shared memory are set within the time limit, \b false otherwise (and the shared memory is not set).
		bool set(TYPE &in, double timeout_s){
			_time_request = ros::Time::now();
			while(_busy){
				if ((ros::Time::now()-_time_request).toSec()>timeout_s){
					return false;
				}
			}
			_busy = true;
			_data = in;
			_busy = false;
			_set = true;
			return true;
		}
		
		bool ever_set(){
			return _set;
		}
		
		
};


}




#endif






