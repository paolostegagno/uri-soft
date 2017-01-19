#include <uri_bridge/bridge_map.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>



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

	unsigned int num_readings = 360;
	double laser_frequency = 40;
	
	//populate the LaserScan message
	sensor_msgs::LaserScan scan;
// 	scan.header.stamp = scan_time;
	scan.header.frame_id = "laser_frame";
	scan.angle_min = -M_PI;
	scan.angle_max = M_PI;
	scan.angle_increment = 2*M_PI / num_readings;
	scan.time_increment = (1 / laser_frequency) / (num_readings);
	scan.range_min = 0.0;
	scan.range_max = 4.0;

	scan.ranges.resize(num_readings);
	scan.intensities.resize(num_readings);
	double angle = scan.angle_min;
	for(unsigned int i = 0; i < num_readings; ++i){
		scan.ranges[i] = std::min(3.5 + 1.3*std::sin(4.5*angle), (double)scan.range_max);
		scan.intensities[i] = 0;
// 		if (angle>-scan.angle_increment/2 && angle<scan.angle_increment/2){
// 			std::cout << "A " << angle << " " << scan.ranges[i] << std::endl;
// 			scan.ranges[i] = 4.5;
// 		}
// 		if (angle>M_PI/2-scan.angle_increment/2 && angle<M_PI/2+scan.angle_increment/2){
// 			std::cout << "A " << angle << " " << scan.ranges[i] << std::endl;
// 			scan.ranges[i] = 2.5;
// 		}
		angle += scan.angle_increment;
	}
	
		std::cout << "a" << std::endl;
	
	uri_bridge::GridMap map(0.05, 0.05, 5.00, 5.00);
		std::cout << "a" << std::endl;

	map.import_scan(scan);
		std::cout << "a" << std::endl;
	
// 	map.print_stats();
// 	int a,b;
// 	double x,y;
// 	
// 	map.coordinates_to_cell(5,-5,a,b);
// 	std::cout << " " << a << " " << b << std::endl;
// 	map.cell_to_coordinates(a,b,x,y);
// 	std::cout << " " << x << " " << y << std::endl;
// 	map.coordinates_to_cell(5.01,-5.01,a,b);
// 	std::cout << " " << a << " " << b << std::endl;
// 	map.cell_to_coordinates(a,b,x,y);
// 	std::cout << " " << x << " " << y << std::endl;
// 	map.coordinates_to_cell(5.02,-5.02,a,b);
// 	std::cout << " " << a << " " << b << std::endl;
// 	map.cell_to_coordinates(a,b,x,y);
// 	std::cout << " " << x << " " << y << std::endl;
// 	map.coordinates_to_cell(5.03,-5.03,a,b);
// 	std::cout << " " << a << " " << b << std::endl;
// 	map.cell_to_coordinates(a,b,x,y);
// 	std::cout << " " << x << " " << y << std::endl;
// 	map.coordinates_to_cell(5.04,-5.04,a,b);
// 	std::cout << " " << a << " " << b << std::endl;
// 	map.cell_to_coordinates(a,b,x,y);
// 	std::cout << " " << x << " " << y << std::endl;
	
// 	map.print();
	map.show_grid_map(5,0);
	
	

}

