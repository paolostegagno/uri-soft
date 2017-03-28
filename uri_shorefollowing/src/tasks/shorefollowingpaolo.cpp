#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_shorefollowing/tasks/shorefollowingpaolo.hpp>

namespace uri_shorefollowing{

  
  
  

ShoreFollowingPaolo::ShoreFollowingPaolo():Task(){
	// You should keep this line on top and put the name of your task in it.
	// Not giving a name to your task will have an unpredictable behavior
	// and most likely will not work.
	_name = "uri_shorefollowing::ShoreFollowingPaolo";
	
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
	
	// how to get the value of an option - use the following syntax:
	// _options["OPTION_NAME"]->getTYPEValue()
	//
	// example:
	// _options["name_option_2"]->getDoubleValue();
	//
}



double ShoreFollowingPaolo::compute_heading_velocity(sensor_msgs::LaserScan &scan){
	
	// find the ray in the middle
	int num_scans = scan.ranges.size();
	int mid_angle_index = num_scans/2;
	double mid_angle = scan.angle_min + (mid_angle_index)*scan.angle_increment;
	
// 	std::cout << "num_scans " << num_scans  << " " << mid_angle_index << " " << mid_angle << " " << scan.angle_min + (mid_angle_index-1)*scan.angle_increment << " " << scan.angle_min + (mid_angle_index+1)*scan.angle_increment << std::endl;
	
	// find the first and last ray considered in the scan
	int laser_steps = 5;
	// compute the max and min angles to be used to find water and land transition
	int plus_minus_lateral_angle_index = std::min( (int)(M_PI/4/scan.angle_increment), mid_angle_index);
	int plus_minus_lateral_angle_index_bool_vector = plus_minus_lateral_angle_index/laser_steps;
	plus_minus_lateral_angle_index = plus_minus_lateral_angle_index_bool_vector*laser_steps; // this is now divisible by laser_steps
	
	// compute corresponding min and max indices and angles
	int min_angle_index = std::max(mid_angle_index-plus_minus_lateral_angle_index, 0);
	int max_angle_index = std::min(mid_angle_index+plus_minus_lateral_angle_index, (int)scan.ranges.size());
	double min_angle = scan.angle_min + ((double)min_angle_index)*scan.angle_increment;
	double max_angle = scan.angle_min + ((double)max_angle_index)*scan.angle_increment;
	
	
	bool ray_points_water[plus_minus_lateral_angle_index_bool_vector*2];
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector; i++){
		ray_points_water[plus_minus_lateral_angle_index_bool_vector-i-1]=true;
		ray_points_water[plus_minus_lateral_angle_index_bool_vector+i]=true;
		int water_rays_plus=0;
		int water_rays_minus=0;
		for (int j = 1; j<=laser_steps; j++){
// 			std::cout << " " << mid_angle_index+j+i*laser_steps << " " << mid_angle_index-j-i*laser_steps;
			if (scan.ranges[mid_angle_index+j+i*laser_steps]>scan.range_max-0.2){
				water_rays_plus++;
			}
			if (scan.ranges[mid_angle_index-j-i*laser_steps]>scan.range_max-0.2){
				water_rays_minus++;
			}
		}
// 		std::cout << " " << water_rays_minus << " " << water_rays_plus << " " << i << std::endl;
		if (water_rays_plus<laser_steps/2){
			ray_points_water[plus_minus_lateral_angle_index_bool_vector+i] = false;
		}
		if (water_rays_minus<laser_steps/2){
			ray_points_water[plus_minus_lateral_angle_index_bool_vector-i-1] = false;
		}
	}
	
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector*2; i++){
		std::cout << ray_points_water[i];
	}
// 	std::cout << std::endl;
	
	
	int first_lw_change = plus_minus_lateral_angle_index_bool_vector;
	double sign = 0.0;
	bool no_lw_transition = true;
	double lw;
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector; i++){
		bool left_left = ray_points_water[plus_minus_lateral_angle_index_bool_vector+i];
		bool left_right = ray_points_water[plus_minus_lateral_angle_index_bool_vector+i-1];
		bool right_right = ray_points_water[plus_minus_lateral_angle_index_bool_vector-i];
		bool right_left = ray_points_water[plus_minus_lateral_angle_index_bool_vector-i-1];
		if (left_left != left_right ){
			first_lw_change = i;
			sign = (double)left_right - (double)left_left;
			no_lw_transition = false;
			break;
		}
		if (right_left != right_right ){
			first_lw_change = -i;
			sign = (double)right_left - (double)right_right;
			no_lw_transition = false;
			break;
		}
	}
	
	


	
// 	int first_land = 0;
// 	if (ray_points_water[0] && !ray_points_water[plus_minus_lateral_angle_index_bool_vector*2-1]){
// 		for(int i=0; i<plus_minus_lateral_angle_index_bool_vector*2; i++){
// 			if (!ray_points_water[i]){
// 				first_land = i;
// 				break;
// 			}
// 		}
// 	}
// 	else if (!ray_points_water[0] && ray_points_water[plus_minus_lateral_angle_index_bool_vector*2-1]){
// 		for(int i=plus_minus_lateral_angle_index_bool_vector*2-1; i>=0; i--){
// 			if (!ray_points_water[i]){
// 				first_land = i;
// 				break;
// 			}
// 		}
// 	}
// 	else {
// 		std::cout << " A " << std::endl;
// 		integral_error = 0.0;
// 		previous_error = plus_minus_lateral_angle_index_bool_vector;
// 		return 0.0;
// 	}
// 	if (first_land > plus_minus_lateral_angle_index_bool_vector) return +0.2;
// 	if (first_land <= plus_minus_lateral_angle_index_bool_vector) return -0.2;
// 
// 	double error = -(double) (plus_minus_lateral_angle_index_bool_vector - first_land) ;
	
	double gain = 1.0;
	
	double error = (double)first_lw_change;
	if (no_lw_transition){
		
		if (ray_points_water[plus_minus_lateral_angle_index_bool_vector] == 0){
			lw = -1;
		} else {
			lw = 1;
		}
		
		error *= lw*prev_sign;
	}
	else {
		prev_sign = sign;
	}
	
	
	
	integral_error += delta_t*error;
	integral_error = std::min(integral_error, 2.0);
	integral_error = std::max(integral_error, -2.0);
	
	double alpha = 0.1;
	if (!init_delta_error){
		delta_error_filter1 = 0.0;
		delta_error = 0.0;
		init_delta_error = true;
	}
	else {
		delta_error_filter1 = (1-alpha)*delta_error_filter1 + alpha*(error - previous_error)/delta_t;
		delta_error = (1-alpha)*delta_error + alpha*delta_error_filter1;
	}
	
	double control = gain*(error*0.02 + integral_error*0.008 + delta_error*0.0010);
	std::cout << " B " << delta_error << " " << error << " " << integral_error << " " << sign << " " << plus_minus_lateral_angle_index_bool_vector  << " " << control << std::endl;
// 	
	previous_error = error;
	
	return control;
	
}





TaskOutput ShoreFollowingPaolo::_run(){
	
	// compute elapsed time since beginning and delta_t since last successful call
	bool terminate = false;
	double elapsed = (ros::Time::now() - start_t).toSec();
	delta_t = elapsed - last_elapsed;	
	
	// check if new laser scan is available. If not, terminate the execution of this _run 
	if (!ls->new_laser_available()){
		return uri::Continue;
	}
	// try to retrieve the scan. If the laser scanner is busy and does not respond in 0.001 seconds, terminate the execution of this _run
	sensor_msgs::LaserScan scan;
	if (!ls->get(scan, 0.001)){
		return uri::Continue;
	}
	
	
	
	
	double heading_velocity = compute_heading_velocity(scan);
	
// 	std::cout << "  heading_velocity " << heading_velocity << std::endl;
	
	
	
	
	
	// if we made it this far, scan contains a new scan! we should decide the new heading based on it
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// PUT HERE CODE FOR DECIDING THE HEADING!!!
	// the lines below simply make the UAV go in circle
	heading_d.heading = heading_d.heading + heading_velocity*delta_t;
	// normalize the heading_d between -M_PI and M_PI
	while (heading_d.heading > M_PI) heading_d.heading = heading_d.heading - 2*M_PI;
	while (heading_d.heading < -M_PI) heading_d.heading = heading_d.heading + 2*M_PI;
	
	// here we set the desired heading in the shared memory - the controller will use it
	desired_heading->set(heading_d, 0.001);
	
	// update last time  we computed the heading
	last_elapsed = elapsed;
	
	// set terminate at true to communicate to the behavior controller to terminate the execution of the task.
	if (terminate){
		return uri::Terminate;
	}
	// but usually, terminate is false.
	return uri::Continue;
}

void ShoreFollowingPaolo::_activate(){
	
	// what do you need to do every time the task is activated?
	last_elapsed = 0.0;
	start_t = ros::Time::now();
	
	delta_error = 0.0;
	delta_error_filter1=0.0;
	init_delta_error = false;
	
	// select first desired heading as the current yaw
	double current_yaw;
	Eigen::Quaterniond current_ori = uav->orientation();
	uri_base::quaternion_to_yaw(current_ori, current_yaw);
	heading_d.heading = current_yaw;
}

void ShoreFollowingPaolo::_deactivate(){
	// what do you need to do every time the task is deactivated?
}

void ShoreFollowingPaolo::get_mandatory_resources(ResourceVector &res){
	
	// to get the resources needed in this task, use the following method:
	//
	// ResourceType res; this declaration should be in the class definition in the header file
	// std::string iint("uri::name_of_the_resource");
	// res = (ResourceType*)res.get_resource_ptr(iint);
	
	
	std::string lint("uri_sensors::LaserScanner");
	ls = (uri_sensors::LaserScanner*)res.get_resource_ptr(lint);
	
	std::string iint("uri_base::SharedMemory<uri_base::Heading>");
	desired_heading = (uri_base::SharedMemory<uri_base::Heading>*)res.get_resource_ptr(iint);
	
	std::string mint("uri_uav::IrisInterface");
	uav = (uri_uav::IrisInterface*)res.get_resource_ptr(mint);
	//
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


