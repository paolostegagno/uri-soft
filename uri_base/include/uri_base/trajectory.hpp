

#include <string>


#include <pluginlib/class_list_macros.h>
// #include <uri_base/shared_memory.hpp>

#include <Eigen/Geometry>

#include <sensor_msgs/LaserScan.h>


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

	
	
	class Pose{
		
		public:
			
			Eigen::Vector3d pos;
			
			Eigen::Quaterniond ori;
			
			Pose(){
				pos(0) = 0.0;
				pos(1) = 0.0;
				pos(2) = 0.0;
				ori.x() = 0.0;
				ori.y() = 0.0;
				ori.z() = 0.0;
				ori.w() = 1.0;
			}
			
			std::string name(){
				return "uri_base::Pose";
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
	
	
	
	
	
	
	
	
	
	class TwoByNMatrix{
		
		double _step;
		double _max;
		double _min;
		int _cell_number;
		
		public:
			
			std::vector<double> _mean_intensities;
			std::vector<double> _measurements_counter;
			
			TwoByNMatrix(){
				_mean_intensities.clear();
				_measurements_counter.clear();
			}
			
			std::string name(){
				return "uri_base::TwoByNMatrix";
			}
			
			void initialize(double step, double max, double min){
				_step = step;
				_max  = max;
				_min = min;
				
				_cell_number = _max/_step;
				
				_mean_intensities.resize(_cell_number);
				_measurements_counter.resize(_cell_number);
				
				for (int i=0; i < _cell_number; i++ ){
					_mean_intensities[i] = 0;
					_measurements_counter[i] = 0;
				}
			}
			
			
			
			void update(sensor_msgs::LaserScan l){
				std::cout << " " << l.intensities.size() << " " << l.ranges.size() << std::endl;
				
				for (int j=0; j < l.intensities.size(); j++ ){
					
					double dist = l.ranges[j];
					double intensity = l.intensities[j];
					int cell_index = ceil(dist/_step);
// 					std::cout << " " << cell_index;
					
					_measurements_counter[cell_index]++;
					_mean_intensities[cell_index] = (_measurements_counter[cell_index]-1)*_mean_intensities[cell_index]/_measurements_counter[cell_index]
																					+ intensity/_measurements_counter[cell_index];
				}
				std::cout << std::endl;
			}
			
			
			
			void print(std::stringstream &ss){
				
				ss << _step << " " << _max << " " << _min << " " << _cell_number;
				for (int j=0; j < _cell_number; j++ ){
					ss << " " << _mean_intensities[j];
				}
				for (int j=0; j < _cell_number; j++ ){
					ss << " " << _measurements_counter[j];
				}
				ss << std::endl;
			}
	};
	
	
	
	
	
	
	
	
	
	class String{
		
		public:
			
			std::string str;
			
			String(){
				str = "";
			}
			
			std::string name(){
				return "uri_base::String";
			}
	};
	
	

}




#endif






