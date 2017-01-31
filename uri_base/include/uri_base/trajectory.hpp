

#include <string>


#include <pluginlib/class_list_macros.h>
// #include <uri_base/shared_memory.hpp>

#include <Eigen/Geometry>




#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__



namespace uri_base {
	
	
	
	
	
	class Heading{
		
		public:
			
			double heading;
			
			Heading(){
				heading = 0.0;
			}
			
			std::string name(){
				return "uri_base::Heading";
			}
	};

	
	
	
	
	
	
	
	class Trajectory{
		
		public:
			
			Eigen::Vector3d pos;
			
			Eigen::Vector3d vel;
			
			Eigen::Vector3d acc;
			
			double yaw;
			
			double yawrate;
			
			Trajectory(){
				pos << 0,0,0;
				vel << 0,0,0;
				acc << 0,0,0;
				yaw = 0;
				yawrate = 0;
			}
			
			std::string name(){
				return "uri_base::Trajectory";
			}
	};
	

}




#endif






