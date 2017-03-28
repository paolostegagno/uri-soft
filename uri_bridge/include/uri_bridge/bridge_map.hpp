
#include <vector>
#include <iostream>

#include <Eigen/Geometry>
#include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>

// #include <cmath>

// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/highgui/highgui.hpp"

#ifndef __BRIDGE_MAP_HPP__
#define __BRIDGE_MAP_HPP__




namespace uri_bridge{
	
	
	
	
	
	class Girder{
		double width;
		double gap;
	};
	
	
	class BridgeMap{
		
	private:
		
		// deck description
		double deck_width;
		double deck_lenght;
		
		// girder description
		int girder_number;
		std::vector<double> girder_width;
		std::vector<double> girder_gap;
		
		
	public:
		
		
		
		BridgeMap(){}
		
		
		void update_girder_number(int gn){
			girder_number = gn;
			girder_width.resize(gn);
			girder_gap.resize(gn+1);
		}
		
		
	};
	
	
};


#endif