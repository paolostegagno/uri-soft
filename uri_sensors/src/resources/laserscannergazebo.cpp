

#include "uri_sensors/resources/laserscannergazebo.hpp"



namespace uri_sensors{


	void LaserScannerGazebo::__init(){
		
		
				std::cout << " LaserScannerGazebo::__init(){ " << std::endl;

		
		_laser_sub = n->subscribe<sensor_msgs::LaserScan>(_options["input_topic"]->getStringValue(), 10, &LaserScannerGazebo::scan_callback, this);
		
// 		_backup = &_las_2;
// 		_active = &_las_1;
		
	}
	
	
	LaserScannerGazebo::LaserScannerGazebo():LaserScanner(){
		
// // 	you can add options here
// 		_options.addBoolOption("optionname_bool", false);
		_options.addStringOption("input_topic", "/default");
// 		_options.addIntOption("optionname_int", 0);
// 		_options.addDoubleOption("optionname_double", 0.0);
		
		
// // 		how to recall an option
// 		bool val_option = _options["optionname_bool"]->getBoolValue();
		
	}
	
	
	void LaserScannerGazebo::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
		
		ros::Time start_t = ros::Time::now();
		while (_backup_busy){
			usleep(100);
			if ( (ros::Time::now() - start_t).toSec() >0.001 ){
				return;
			}
		}
		
		_backup_busy = true;
		
		_backup->angle_min = msg->angle_min;
		_backup->angle_max = msg->angle_max;
		_backup->header = msg->header;
		_backup->angle_increment = msg->angle_increment;
		_backup->range_min = msg->range_min;
		_backup->range_max = msg->range_max;
		_backup->scan_time = msg->scan_time;
		_backup->time_increment = msg->time_increment;
		_backup->ranges = msg->ranges;
		_backup->intensities = msg->intensities;
		
		_backup_busy = false;
		
		_new_laser_arrived = true;
		
// // 		std::cout << 	_backup->angle_min << " "
// // 		<< _backup->angle_max << " "
// // 		<< _backup->header << " "
// // 		<< _backup->angle_increment << " "
// // 		<< _backup->range_min << " "
// // 		<< _backup->range_max << " "
// // 		<< _backup->scan_time << " "
// // 		<< _backup->time_increment << " "
// // 		<< _backup->ranges.size() << " " 
// // 		<< _backup->intensities.size() << " " << std::endl;

// 		_backup->intensities = msg->intensities;
		
		
	}
	
	
	
// 	bool LaserScannerGazebo::set(sensor_msgs::LaserScan& scan, double timeout){
	
// 	}
	
	PLUGINLIB_EXPORT_CLASS(uri_sensors::LaserScannerGazebo, uri::Resource)

	

}; // end namespace




