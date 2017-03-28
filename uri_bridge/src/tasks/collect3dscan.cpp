#include <iostream>
#include <fstream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_bridge/tasks/collect3dscan.hpp>

namespace uri_bridge{

  
  
  

Collect3DScan::Collect3DScan():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_bridge::Collect3DScan";
	
	// The class Task contains a field _options which you can use freely.
	// You can add (and pass options through the configuration file) with the following lines.
	// [ format: _options.addTYPEOption(OPTION_NAME, DEFAULT_VALUE); ]
	//
	// _options.addIntOption("name_option_1",15);
	// _options.addDoubleOption("name_option_2",0.05);
	// _options.addBoolOption("name_option_3",false);
	// _options.addStringOption("name_option_2","default_value");
	//
	// Note that options are only of those types listed here.
	// The first parameter in the above lines is the name, while the second parameter is the default value.
	// If the option is specified in the config file, the value of the option is automatically updated.
	// If the option specfied is not in the config_file, the option will have the default value
	// note that the options are updated after the execution of this constructor, so any option in this
	// construction will have its default value.
}

TaskOutput Collect3DScan::_run(){
	
	bool terminate = false;
	// check if new laser scan is available. If not, terminate the execution of this _run 
	if (!ls->new_laser_available()){
		return uri::Continue;
	}
	// try to retrieve the scan. If the laser scanner is busy and does not respond in 0.001 seconds, terminate the execution of this _run
	sensor_msgs::LaserScan scan;
	
	if (!ls->get(scan, 0.001)){
		return uri::Continue;
	}
	
	// take position and orientation of the UAV
	_position = uav->position();
	_orientation = uav->orientation();
	double r, p, y;
	uri_base::quaternion_to_rpy(_orientation, r, p, y);
	
	
	// declare some variables and create rotation matrix
	int num_readings = scan.ranges.size();
	double angle = scan.angle_min;
	double x_p, y_p;
	double range;
	Eigen::Matrix3d R = rpy_to_rot(M_PI/2, -M_PI/2, y);
	if (not first_yaw_selected) {
		new_2D_scan.position = _position;
// 		first_yaw = y;
// 		if (first_yaw<0) first_yaw += 2*M_PI; 
// 		last_yaw = y + M_PI;
// 		if (last_yaw<0) last_yaw += 2*M_PI; 
		first_yaw_selected = true;
		previous_yaw = y;
// 		std::cout << "fy " << first_yaw << " ly " last_yaw;

	}
	
	fout << _position[0] << " " << _position[1] << " " << _position[2] 
					<< " " << r << " " << p << " " << y << " " << scan.header.seq ;
	
	// expand the pointcloud to accomodate num_readings new points
	int num_points = pc.points.size();
	pc.points.resize(num_points+num_readings);
	int counter=0;
	
	
	// create a 3D point out of each point in the laser scan
	for(unsigned int i = 0; i < num_readings; ++i){
		
		// traslate polar coordinates into cartesian coordinates
		range = scan.ranges[i];
		x_p = range * std::cos(angle);
		y_p = range * std::sin(angle);
		angle += scan.angle_increment;
		
		fout << " " << x_p << " " << y_p; 
		
		
		Eigen::Vector3d p;
		p(0) = x_p; p(1) = y_p; p(2) = 0.0;
		double pnorm = p.norm();
		if (pnorm>0.3 /*&& pnorm <9.95*/){
			Eigen::Vector3d q = R*p;
			if (q(2)>-0.6 && q(2)<0.1){
				pc.points[num_points + counter].x = q(0);
				pc.points[num_points + counter].y = q(1);
				pc.points[num_points + counter].z = q(2);
				counter++;
				
				double at2 = atan2(q(1), q(0));
				double nor = sqrt(q(1)*q(1)+q(0)*q(0));
				int index_in_new_scan = (int)((at2+M_PI)/new_2D_scan.angle_increment + 0.5);
				if (index_in_new_scan >= new_2D_scan.ranges.size()){
					index_in_new_scan -= new_2D_scan.ranges.size();
				}
				double old = new_2D_scan.ranges[index_in_new_scan];
				new_2D_scan.ranges[index_in_new_scan] = std::min(old, nor);
			}
		}
	}
	pc.points.resize(num_points+counter);
	
	fout << std::endl;
	
	// compute condition for finishing the scan collection.
	double delta_yaw = y-previous_yaw;
	while (delta_yaw < -M_PI){ delta_yaw += 2*M_PI; }
	while (delta_yaw > M_PI){ delta_yaw -= 2*M_PI; }
	traveled_yaw += delta_yaw;
// 	std::cout << traveled_yaw << " " << delta_yaw << std::endl;
// 	if (y<0) { y += 2*M_PI; }
	if (traveled_yaw > M_PI){
		
		lsc->set(new_2D_scan, 0.2);
		
// 		pcl->set(pc, 0.2);
		return uri::Terminate;
	}
	previous_yaw = y;
	return uri::Continue;
}



void Collect3DScan::_initialize(){
	pc.header.frame_id ="pc";
	counter = 1;
}


void Collect3DScan::_activate(){
	// what do you need to do every time the task is activated?
	
	std::stringstream namefile;
	namefile << "/home/paolos/exploration_data/scan_" << counter << ".txt"; 
	fout.open (namefile.str().c_str(), std::fstream::out );
	
	std::cout << "################################################ " << std::endl;
	std::cout << "    " << namefile.str().c_str() << std::endl;
	
	counter++;
	first_yaw_selected = false;
// 	new_scan_polar.clear();
	
	unsigned int num_readings = 360;
	double laser_frequency = 40;
	new_2D_scan.header.frame_id = "laser_frame";
	new_2D_scan.angle_min = -M_PI;
	new_2D_scan.angle_max = M_PI;
	new_2D_scan.angle_increment = 2*M_PI / num_readings;
	new_2D_scan.time_increment = (1 / laser_frequency) / (num_readings);
	new_2D_scan.range_min = 0.0;
	new_2D_scan.range_max = 10.0;
	new_2D_scan.ranges.resize(num_readings);
// 	num_rays_per_scan.resize(num_readings);
	for (int i=0; i< num_readings; i++){
		new_2D_scan.ranges[i]=new_2D_scan.range_max;
// 		num_rays_per_scan[i]=0.0;
	}
	traveled_yaw = 0.0;
}

void Collect3DScan::_deactivate(){
	fout.close();
	// what do you need to do every time the task is deactivated?
}



void Collect3DScan::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	std::string lint("uri_sensors::LaserScanner");
	ls = (uri_sensors::LaserScanner*)res.get_resource_ptr(lint);
	
	std::string mint("uri_uav::IrisInterface");
	uav = (uri_uav::IrisInterface*)res.get_resource_ptr(mint);
	
// 	std::string pint("uri_base::SharedMemory<uri_bridge::PointCloud>");
// 	pcl = (uri_base::SharedMemory<uri_bridge::PointCloud>*)res.get_resource_ptr(pint);
	
	std::string sint("uri_base::SharedMemory<uri_bridge::LaserScan>");
	lsc = (uri_base::SharedMemory<uri_bridge::LaserScan>*)res.get_resource_ptr(sint);
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


