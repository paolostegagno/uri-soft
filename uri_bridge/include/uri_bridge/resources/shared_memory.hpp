
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <uri_base/shared_memory.hpp>

namespace uri_bridge{
	
	
	class LaserScan: public sensor_msgs::LaserScan {
		
		public:
			
			Eigen::Vector3d position;
			
			std::string name(){
				return "uri_bridge::LaserScan";
			}
		
	};
	
	
	
	
	
	class PointCloud: public sensor_msgs::PointCloud {
		
		public:
			std::string name(){
				return "uri_bridge::PointCloud";
			}
		
	};
	
	
	
	
	PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<uri_bridge::LaserScan>, uri::Resource)
	
	PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<uri_bridge::PointCloud>, uri::Resource)
	
};