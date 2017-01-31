

#include "uri_sensors/resources/laserscanner.hpp"



namespace uri_sensors{


	void LaserScanner::_init(){
		
		_exchange_active_backup_tmr = n->createTimer(ros::Duration(0.001), &LaserScanner::_exchange_active_backup_callback, this);
		_exchange_active_backup_tmr.start();
		
		__init();
	}
	
	
	LaserScanner::LaserScanner(){
		
		_name = "uri_sensors::LaserScanner";
		
// // 	you can add options here
// 		_options.addBoolOption("optionname_bool", false);
// 		_options.addStringOption("optionname_str", "default");
// 		_options.addIntOption("optionname_int", 0);
// 		_options.addDoubleOption("optionname_double", 0.0);
		
		
		_active = &_las_1;
		_backup = &_las_2;
		
		_backup_busy = false;
		_active_busy = false;
		_new_laser_arrived = false;
		_new_laser_available = false;
		
// // 		how to recall an option
// 		bool val_option = _options["optionname_bool"]->getBoolValue();
		
	}
	
	
	
	void LaserScanner::_exchange_active_backup_callback(const ros::TimerEvent&){
		if (!_new_laser_arrived){
			return;
		}
		while(_backup_busy or _active_busy){
			usleep(100);
		}
		_backup_busy = true;
		_active_busy = true;
		sensor_msgs::LaserScan* temp = _active;
		_active = _backup;
		_backup = temp;
		_new_laser_arrived = false;
		_new_laser_available = true;
		_backup_busy = false;
		_active_busy = false;
	}
	
	
	
	bool LaserScanner::get(sensor_msgs::LaserScan& scan, double timeout){
		
		ros::Time  start_t = ros::Time::now();
		
		while (_active_busy and ((ros::Time::now()-start_t).toSec() < timeout) ){
			usleep(100);
		}
		if (_active_busy){
			return false;
		}
		_active_busy = true;
		scan = *_active;
		_new_laser_available = false;
		_active_busy = false;
		
		return true;
	}
	
	


}; // end namespace




