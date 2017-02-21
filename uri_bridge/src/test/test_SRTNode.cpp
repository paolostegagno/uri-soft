#include <uri_bridge/bridge_map.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <fstream>

// int main(void){
// 	
// 	uri_bridge::GridMap map;
// 	
// // 	map.print();
// 	std::cout << map;
// 	
// 	
// 	return 0;
// }




int main(int argc, char** argv){
	
	
   ros::init(argc, argv, "test");
	 ros::NodeHandle n;
	
	
	std::fstream fin("/home/paolos/Desktop/data_ridot/mapscan_polar.txt", std::fstream::in);
	
	

	unsigned int num_readings = 360;
	
	fin >> num_readings;
	
	std::cout << "num readings " << std::endl;
	
	double laser_frequency = 40;
	
	//populate the LaserScan message
	sensor_msgs::LaserScan scan;
	
	
// 	scan.header.stamp = scan_time;
	scan.header.frame_id = "laser_frame";
	scan.angle_min = -M_PI;
	scan.angle_max = M_PI;
	
	fin >> scan.angle_min;
	fin >> scan.angle_max;
	
	
	scan.angle_increment = 2*M_PI / num_readings;
	scan.time_increment = (1 / laser_frequency) / (num_readings);
	scan.range_min = 0.0;
	scan.range_max = 10.0;

	scan.ranges.resize(num_readings);
// 	scan.intensities.resize(num_readings);
	double angle = scan.angle_min;
	for(unsigned int i = 0; i < num_readings; ++i){
		fin >> scan.ranges[i];
	}
	
	Eigen::Vector2d pos;
	pos(0) = 0.5;
	pos(1) = -0.5;
	uri_bridge::GridMapParams gmp(pos,0.05, 0.05, 10.80, 10.80, 0.91, 0.5);
// 	uri_bridge::SRTNode node(pos, gmp, scan);
	
// 	node. ;
	
	
// // 		std::cout << "a" << std::endl;
// 	ros::Time start = ros::Time::now();
// 	uri_bridge::SRTNode map(0.05, 0.05, 10.80, 10.80);
// // 		std::cout << "a" << std::endl;
// 
// 	map.import_scan(scan, 0.901);
// // 		std::cout << "a" << std::endl;
// 	std::cout << ( ros::Time::now() - start ).toSec() << std::endl ;
// 	
// 
// // 	map.print();
// 	map.show_grid_map_color(21,0);
// 	map.print_stats();
// 	
	

}

