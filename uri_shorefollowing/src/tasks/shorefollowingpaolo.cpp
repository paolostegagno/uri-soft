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
	
	vs = nullptr;
}




double ShoreFollowingPaolo::compute_heading_velocity(sensor_msgs::LaserScan &scan){
	
	out_file << ros::Time::now().toSec() - init_time;
	
	
	
	// find the ray in the middle
	int num_scans = scan.ranges.size();
	int mid_angle_index = num_scans/2;
	double mid_angle = scan.angle_min + (mid_angle_index)*scan.angle_increment;
	
// std::cout << "num_scans " << num_scans  << " " << mid_angle_index << " " << mid_angle << " " << scan.angle_min + (mid_angle_index-1)*scan.angle_increment << " " << scan.angle_min + (mid_angle_index+1)*scan.angle_increment << std::endl;
	
// find the first and last ray considered in the scan
	int laser_steps = 5;
	// compute the max and min angles to be used to find water and land transition
	int plus_minus_lateral_angle_index = std::min( (int)(max_angle_terrain/scan.angle_increment), mid_angle_index);
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
			if (scan.ranges[mid_angle_index+j+i*laser_steps]>scan.range_max-0.2){
				water_rays_plus++;
			}
			if (scan.ranges[mid_angle_index-j-i*laser_steps]>scan.range_max-0.2){
				water_rays_minus++;
			}
		}
		if (water_rays_plus<laser_steps/2){
			ray_points_water[plus_minus_lateral_angle_index_bool_vector+i] = false;
		}
		if (water_rays_minus<laser_steps/2){
			ray_points_water[plus_minus_lateral_angle_index_bool_vector-i-1] = false;
		}
	}
	
	// water ray points on file
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector*2; i++){
		std::cout << ray_points_water[i];
		out_file << " " << ray_points_water[i];
	}
	std::cout << std::endl;
	
	
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
	
	// here compute the error 
	double error = (double)first_lw_change;
	if (no_lw_transition){
		if (ray_points_water[plus_minus_lateral_angle_index_bool_vector] == 0){ lw = -1;}
		else { lw = 1;}
		error *= lw*prev_sign;
	}
	else { prev_sign = sign;}
	
	return line_following_controller((double)first_lw_change);
}






double ShoreFollowingPaolo::compute_heading_velocity_with_intensity_model(sensor_msgs::LaserScan &scan){
	
	out_file << ros::Time::now().toSec() - init_time;
	
	// find the ray in the middle
	int num_scans = scan.ranges.size();
	int mid_angle_index = num_scans/2;
	double mid_angle = scan.angle_min + (mid_angle_index)*scan.angle_increment;
	
// std::cout << "num_scans " << num_scans  << " " << mid_angle_index << " " << mid_angle << " " << scan.angle_min + (mid_angle_index-1)*scan.angle_increment << " " << scan.angle_min + (mid_angle_index+1)*scan.angle_increment << std::endl;
	
// find the first and last ray considered in the scan
	int laser_steps = 5;
	// compute the max and min angles to be used to find water and land transition
	int plus_minus_lateral_angle_index = std::min( (int)(max_angle_terrain/scan.angle_increment), mid_angle_index);
	int plus_minus_lateral_angle_index_bool_vector = plus_minus_lateral_angle_index/laser_steps;
	plus_minus_lateral_angle_index = plus_minus_lateral_angle_index_bool_vector*laser_steps; // this is now divisible by laser_steps
	
	// compute corresponding min and max indices and angles
	int min_angle_index = std::max(mid_angle_index-plus_minus_lateral_angle_index, 0);
	int max_angle_index = std::min(mid_angle_index+plus_minus_lateral_angle_index, (int)scan.ranges.size());
	double min_angle = scan.angle_min + ((double)min_angle_index)*scan.angle_increment;
	double max_angle = scan.angle_min + ((double)max_angle_index)*scan.angle_increment;
	
	// @@@@@@@@@@@@@ start here first method @@@@@@@@@@@@@@
	// find if a considered group of laser rays are water or land depending on invalid measurements only
	bool ray_points_water[plus_minus_lateral_angle_index_bool_vector*2];
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector; i++){
		ray_points_water[plus_minus_lateral_angle_index_bool_vector-i-1]=true;
		ray_points_water[plus_minus_lateral_angle_index_bool_vector+i]=true;
		int water_rays_plus=0;
		int water_rays_minus=0;
		for (int j = 1; j<=laser_steps; j++){
			if (scan.ranges[mid_angle_index+j+i*laser_steps]>scan.range_max-0.2){
				water_rays_plus++;
			}
			if (scan.ranges[mid_angle_index-j-i*laser_steps]>scan.range_max-0.2){
				water_rays_minus++;
			}
		}
		if (water_rays_plus<laser_steps/2){
			ray_points_water[plus_minus_lateral_angle_index_bool_vector+i] = false;
		}
		if (water_rays_minus<laser_steps/2){
			ray_points_water[plus_minus_lateral_angle_index_bool_vector-i-1] = false;
		}
	}
	
	// and save the results on file
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector*2; i++){
// 		std::cout << ray_points_water[i];
		out_file << " " << ray_points_water[i];
	}
	// @@@@@@@@@@@@@ end here first method @@@@@@@@@@@@@@
	
	// put a hugee number as separator between the two results of first and second method
	out_file << " " << 100000;
	
	// @@@@@@@@@@@@@ start here second method @@@@@@@@@@@@@@
	// find if a considered group of laser rays are water or land depending invalid measurements and itensity model
	bool ray_points_water_from_intensity[plus_minus_lateral_angle_index_bool_vector*2];
	bool intensity_errors[plus_minus_lateral_angle_index_bool_vector*2*laser_steps];
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector; i++){
		ray_points_water_from_intensity[plus_minus_lateral_angle_index_bool_vector-i-1]=true;
		ray_points_water_from_intensity[plus_minus_lateral_angle_index_bool_vector+i]=true;
		int water_rays_plus=0;
		int water_rays_minus=0;
		double intensity_error_plus;
		double intensity_error_minus;
		for (int j = 1; j<=laser_steps; j++){
			if (scan.ranges[mid_angle_index+j+i*laser_steps]>scan.range_max-0.2){ // if invalid consider it as water
				water_rays_plus++;
			}
			else { // if not invalid, check the intensity model
// 				std::cout << "a" << std::endl;
				intensity_error_plus = int_mod.distance_from_mean_intensity(scan.ranges[mid_angle_index+j+i*laser_steps], scan.intensities[mid_angle_index+j+i*laser_steps]);
				if (intensity_error_plus <0){
					water_rays_plus++;
				}
// 				std::cout << "b" << std::endl;
			}
			if (scan.ranges[mid_angle_index-j-i*laser_steps]>scan.range_max-0.2){
				water_rays_minus++;
			}
			else { // if not invalid, check the intensity model
// 				std::cout << "c" << std::endl;
				intensity_error_minus = int_mod.distance_from_mean_intensity(scan.ranges[mid_angle_index-j-i*laser_steps], scan.intensities[mid_angle_index-j-i*laser_steps]);
				if (intensity_error_minus <0){
					water_rays_minus++;
				}
// 				std::cout << "d" << std::endl;
			}
		}
		if (water_rays_plus<laser_steps/2){
			ray_points_water_from_intensity[plus_minus_lateral_angle_index_bool_vector+i] = false;
		}
		if (water_rays_minus<laser_steps/2){
			ray_points_water_from_intensity[plus_minus_lateral_angle_index_bool_vector-i-1] = false;
		}
	}
	
	// and save the results on file
	for(int i=0; i<plus_minus_lateral_angle_index_bool_vector*2; i++){
// 		std::cout << ray_points_water[i];
		out_file << " " << ray_points_water_from_intensity[i];
	}
	// @@@@@@@@@@@@@ end here second method @@@@@@@@@@@@@@
	
	
	
	
	
	
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
	
	
	
// 	double gain = 1.0;
	
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
// 	
// 	
// 	
// 	integral_error += delta_t*error;
// 	integral_error = std::min(integral_error, 2.0);
// 	integral_error = std::max(integral_error, -2.0);
// 	
// 	double alpha = 0.1;
// 	if (!init_delta_error){
// 		delta_error_filter1 = 0.0;
// 		delta_error = 0.0;
// 		init_delta_error = true;
// 	}
// 	else {
// 		delta_error_filter1 = (1-alpha)*delta_error_filter1 + alpha*(error - previous_error)/delta_t;
// 		delta_error = (1-alpha)*delta_error + alpha*delta_error_filter1;
// 	}
// 	
// 	double control = gain*(error*0.02 + integral_error*0.008 + delta_error*0.0010);
// // 	std::cout << " " << delta_error << " " << error << " " << integral_error << " " << sign << " " << plus_minus_lateral_angle_index_bool_vector  << " " << control << std::endl;
// 	out_file << " " << delta_error << " " << error << " " << integral_error << " " << sign << " " << plus_minus_lateral_angle_index_bool_vector  << " " << control << std::endl;
// // 	
// 	previous_error = error;
// 	
// 	return control;
	
	return line_following_controller((double)first_lw_change);
	
}




double ShoreFollowingPaolo::line_following_controller(double error){
	
	double gain = 1.0;
	
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
// 	std::cout << " " << delta_error << " " << error << " " << integral_error << " " << sign << " " << plus_minus_lateral_angle_index_bool_vector  << " " << control << std::endl;
// 	out_file << " " << delta_error << " " << error << " " << integral_error << " " << sign << " " << plus_minus_lateral_angle_index_bool_vector  << " " << control << std::endl;
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
	
	
	
	// retrieve intensoty model from the shared memory
// 	if (not intensity_model->ever_set()){
// 		ROS_ERROR("Intensity Model never initialized.");
// 	}
// 	else {
// // 		ROS_INFO("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Intensity Model was initialized!!!");
// 		if (intensity_model->get(int_mod, 0.01)){
// 			ROS_INFO("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Intensity Model was initialized and retrieved!!!");
// 		}
// 		else {
// 			ROS_ERROR("Cannot retrieve Intensity Model.");
// 		}
// 	}
// 	
// 	
// 	double heading_velocity = compute_heading_velocity_with_intensity_model(scan);
	
	double heading_velocity = compute_heading_velocity(scan);
	
// 	std::cout << "  heading_velocity " << heading_velocity << std::endl;
	
	
	
	
	
	// if we made it this far, scan contains a new scan! we should decide the new heading based on it

	heading_d.heading = heading_d.heading + heading_velocity*delta_t;
	// normalize the heading_d between -M_PI and M_PI
	while (heading_d.heading > M_PI) heading_d.heading = heading_d.heading - 2*M_PI;
	while (heading_d.heading < -M_PI) heading_d.heading = heading_d.heading + 2*M_PI;
	
	// here we set the desired heading in the shared memory - the controller will use it
	desired_heading->set(heading_d, 0.001);
	
	// update last time  we computed the heading
	last_elapsed = elapsed;
	
	out_file << std::endl;

	
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
	
	flight_height = 2.0;
	max_angle_terrain = std::acos(flight_height/60.0);
	angle_noise_threshold = 0.17;
	max_angle_terrain -= angle_noise_threshold; 

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
	desired_heading = (uri_base::SharedMemory<uri_base::Heading>*)res.get_resource_ptr(iint, "desired_heading");
	
	std::string mint("uri_uav_resources::IrisInterface");
	uav = (uri_uav_resources::IrisInterface*)res.get_resource_ptr(mint);
	
	std::string vint("uri_sensors::VideoStream");
	vs = (uri_sensors::VideoStream*)res.get_resource_ptr(vint, false);
	
	
// 	std::string imint("uri_base::SharedMemory<uri_base::TwoByNMatrix>");
// 	intensity_model = (uri_base::SharedMemory<uri_base::TwoByNMatrix>*)res.get_resource_ptr(imint, "intensity_model");
	
	
	// if you have put res in the header file, you'll be able to use it in any other method of this class (except fo the constructor, which is executed first)
}

};


