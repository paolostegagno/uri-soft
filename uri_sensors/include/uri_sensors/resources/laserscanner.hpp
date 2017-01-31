
#include <string>
#include <thread>

#include <pluginlib/class_list_macros.h>
#include <uri_core/resource.hpp>

#include <sensor_msgs/LaserScan.h>

// #include <Eigen/Geometry>


#ifndef __LASERSCANNER_HPP__
#define __LASERSCANNER_HPP__


using namespace uri;

namespace uri_sensors {
	
	
/// @brief This is a base class for a laser scanner Resource.
/// @details All other laser scanner classes should be derived by this one to have a common interface.
/// However, it is not possible to instantiate directly this class because it has some pure virtual functions.
class LaserScanner: public Resource{
	
		sensor_msgs::LaserScan scan;
		
// 		/// @brief Madatory initialization method.
// 		/// @details this method is called once after the instantiation is created. You can use it to initialize all your private variables
		void _init();
		
		
		
		// put here all your private variables
		
		ros::Timer _exchange_active_backup_tmr;
		
		void _exchange_active_backup_callback(const ros::TimerEvent&);
		
		
		
	protected:
		
		virtual void __init()=0;

		
		sensor_msgs::LaserScan _las_1;
		sensor_msgs::LaserScan _las_2;
		
		sensor_msgs::LaserScan* _active;
		sensor_msgs::LaserScan* _backup;
		
		bool _backup_busy;
		bool _active_busy;
		
		bool _new_laser_arrived;
		
		bool _new_laser_available;
		
		// if you need a thread, you can use the c++11 standard 
		// std::thread* getcharacters;
		
		
		
		
		
	public:
		
		/// @brief Constructor
		/// @details This method does not build anything.
		LaserScanner();
		
		
		/// @brief Get function.
		/// @details This function gets the scan within the specified timeout.
		/// @param[out] &scan the latest laser scan is copied here.
		/// @param[in] timeout maximum time in seconds to wait for the scan.
		/// @return \b true if the scan is copied within the timeout, \b false otherwise.
		bool get(sensor_msgs::LaserScan &scan, double timeout);
		
		
		/// @brief Whether or not a new laser is available since the last get function.
		bool new_laser_available(){
			return _new_laser_available;
		}
		
		/// @brief Set function.
		/// @details This function sets the scan within the specified timeout.
		/// This is a pure virtual method. Each derived class must implement this method
		/// @param[in] &scan the latest laser scan to be copied.
		/// @param[in] timeout maximum time in seconds to wait for setting the scan.
		/// @return \b true if the scan is set within the timeout, \b false otherwise.
// 		virtual bool set(sensor_msgs::LaserScan &scan, double timeout)=0;

		
};
} // end namespace uri_sensors





#endif






