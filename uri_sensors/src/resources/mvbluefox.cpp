

#include "uri_sensors/resources/mvbluefox.hpp"



namespace uri_sensors{


	void MVBluefox::_init(){
		
		
		water_points.push_back(cv::Point(100,100));
		water_points.push_back(cv::Point(110,100));
		water_points.push_back(cv::Point(120,100));
		water_points.push_back(cv::Point(130,100));
		water_points.push_back(cv::Point(140,100));
		land_points.push_back(cv::Point(150,100));
		land_points.push_back(cv::Point(160,100));
		land_points.push_back(cv::Point(170,100));
		land_points.push_back(cv::Point(180,100));
		land_points.push_back(cv::Point(190,100));

		
		video = new cv::VideoCapture(_options["filename"]->getStringValue());
		if(!video->isOpened()) { // check if we succeeded
// 			std::cout << "VIDEO NOT OPEN!! VIDEO NOT OPEN!! VIDEO NOT OPEN!! VIDEO NOT OPEN!! VIDEO NOT OPEN!!" << std::endl;
			ROS_ERROR("  Cannot open video file %s.", _options["filename"]->getStringValue().c_str());
		}
		else{
			for (int i =0; i<_options["skip_frames"]->getIntValue(); i++){
				video->grab();
			}
			_exchange_active_backup_tmr = n->createTimer(ros::Duration(1.0/_options["framerate"]->getDoubleValue()), &MVBluefox::_video_callback, this);
			_exchange_active_backup_tmr.start();
		}
	}
	
	
	MVBluefox::MVBluefox(){
		
		_name = "uri_sensors::MVBluefox";
		
// // 	you can add options here
// 		_options.addBoolOption("optionname_bool", false);
// 		_options.addIntOption("optionname_int", 0);
// 		_options.addDoubleOption("optionname_double", 0.0);
		_options.addStringOption("filename", "/home/paolos/shore_following_data/video/2017_0719_151630_002.MP4");
		_options.addDoubleOption("framerate", 30);
		_options.addIntOption("skip_frames", 200);
		
		
// 		_active = &_las_1;
// 		_backup = &_las_2;
// 		
// 		_backup_busy = false;
// 		_active_busy = false;
// 		_new_laser_arrived = false;
// 		_new_laser_available = false;
// 		
// // // 		how to recall an option
// // 		bool val_option = _options["optionname_bool"]->getBoolValue();
// 		
	}
	
	
	
	void MVBluefox::_video_callback(const ros::TimerEvent&){
		

		cv::Mat frame;
		video->read(frame);
		
		cv::namedWindow("video",1);
		
// 		cvtColor(frame, edges, CV_BGR2GRAY);
// 		GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
//         Canny(edges, edges, 0, 30, 3);
		
		cv::Size size(640,360);//the dst image size,e.g.100x100
		cv::Mat dst;//dst image
		cv::resize(frame,dst,size);//resize image
		
		for (int i=0; i<land_points.size(); i++){
			cv::circle(dst, land_points[i], 1, cv::Scalar(0,0,255), 3);
		}
		for (int i=0; i<water_points.size(); i++){
			cv::circle(dst, water_points[i], 1, cv::Scalar(255,0,0), 3);
		}
		
		cv::imshow("video", dst);
		cv::waitKey(30);
		
// 		if (!_new_laser_arrived){
// 			return;
// 		}
// 		while(_backup_busy or _active_busy){
// 			usleep(100);
// 		}
// 		_backup_busy = true;
// 		_active_busy = true;
// 		sensor_msgs::LaserScan* temp = _active;
// 		_active = _backup;
// 		_backup = temp;
// 		_new_laser_arrived = false;
// 		_new_laser_available = true;
// 		_backup_busy = false;
// 		_active_busy = false;
	}
	
	
	
// 	bool MVBluefox::get(sensor_msgs::LaserScan& scan, double timeout){
// 		
// 		ros::Time  start_t = ros::Time::now();
// 		
// 		while (_active_busy and ((ros::Time::now()-start_t).toSec() < timeout) ){
// 			usleep(100);
// 		}
// 		if (_active_busy){
// 			return false;
// 		}
// 		_active_busy = true;
// 		scan = *_active;
// 		_new_laser_available = false;
// 		_active_busy = false;
// 		
// 		return true;
// 	}
	
	
	
	PLUGINLIB_EXPORT_CLASS(uri_sensors::MVBluefox, uri::Resource)


}; // end namespace




