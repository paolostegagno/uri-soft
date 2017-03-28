#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_bridge/behavior_controllers/srtexploration.hpp>

namespace uri_bridge{

  
  
  

SRTExploration::SRTExploration():BehaviorController()/*:_name(nm)*/{
	_name = "uri_bridge::SRTExploration";
	_ls_ready = false;
	wp_counter = 0;
	back_path.clear();
	_backtracking = false;
	_savedata = true;
}




TaskOutput SRTExploration::__run(){
	
	// at beginning, no behavior is selected. Select here start behavior
	if (_next_active_behavior == NULL && _active_behavior == NULL){
		_next_active_behavior = behavior("Takeoff");
		return uri::Continue;
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Takeoff")){
		if (_active_behavior->terminate()){
			_next_active_behavior = behavior("Goto");
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Goto")){
		if (_active_behavior->terminate()){
			behavior("Hover")->set_option_double("Countdown", "uri_uav::Hover", 3.0);
			_next_active_behavior = behavior("Hover");
		}
	}
	
	if (_active_behavior == behavior("Hover")){
		if (_active_behavior->terminate()){
			if (!_backtracking){
				behavior("Collect3DScan")->set_option_double("YawRate", "uri_uav::Hover", 0.2);
				behavior("Collect3DScan")->set_option_double("Countdown", "uri_uav::Hover", 2000000.0);
				_next_active_behavior = behavior("Collect3DScan");
			}
			else {
				
				srt.set_current_node(back_path[0]);
				back_path.erase(back_path.begin());
				
				// if the next waypoint to visit is again a pre-existing node
				if (back_path.size()>0){
					cv::Point2i nwp;
					double x, y;
					// set the center of the next node as waypoint and enable Goto
					behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", srt.node()[back_path[0]]->params.p(0));
					behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", srt.node()[back_path[0]]->params.p(1));
					std::cout << "NEW BACKTRACK !!! " << back_path[0] << " " << x << " " << y << std::endl;
					_next_active_behavior = behavior("Goto");
				}
				// otherwise say the laser is ready (although it is not a new laser)
				else{
					_ls_ready = true;
				}
			}
		}
	}
	
	// exit from behavior takeoff only when requested by such behavior
	if (_active_behavior == behavior("Collect3DScan")){
		if (_active_behavior->terminate()){
			behavior("Hover")->set_option_double("YawRate", "uri_uav::Hover", 0.0);
			behavior("Hover")->set_option_double("Countdown", "uri_uav::Hover", 2000000.0);
			_next_active_behavior = behavior("Hover");
			_ls_ready = true;
			return uri::Continue;
		}
	}
	
	if (_ls_ready){
		
		// if not exit from a waypoint, add a new node
		if (!_backtracking){
			uri_bridge::LaserScan scan;
			ls->get(scan, 0.1);
			Eigen::Vector2d pos(scan.position(0), scan.position(1));
			uri_bridge::SRTNodeParams gmp(pos, 0.05, 0.05, 11.00, 11.00, 0.81, 0.5);
			srt.add_node(gmp, (sensor_msgs::LaserScan)scan );
		}
		else {
			_backtracking = false;
		}
		
		// compute the next waypoint
		cv::Point2i nwp = srt.compute_next_waypoint();
		
		// TODO the switch has to go!
		switch (wp_counter){
			case 0: nwp.x = 394; nwp.y = 238; break;
			case 1: nwp.x = 176; nwp.y = 386; break;
			case 2: nwp.x = 186; nwp.y = 386; break;
			case 3: nwp.x = 387; nwp.y = 200; break;
			case 4: back_path.push_back(3);
							_backtracking = true;
							srt.current_node()->cartesian_global_to_cell(srt.node()[back_path[0]]->params.p(0), srt.node()[back_path[0]]->params.p(1), nwp.y, nwp.x);
							break;
			case 5: nwp.x = 118; nwp.y = 320; break;
			case 6: back_path.push_back(3);
							_backtracking = true;
							srt.current_node()->cartesian_global_to_cell(srt.node()[back_path[0]]->params.p(0), srt.node()[back_path[0]]->params.p(1), nwp.y, nwp.x);
							break;
			case 7: nwp.x = 102; nwp.y = 92; break;
			case 8: back_path.push_back(3);
							back_path.push_back(2);
							back_path.push_back(1);
							_backtracking = true;
							srt.current_node()->cartesian_global_to_cell(srt.node()[back_path[0]]->params.p(0), srt.node()[back_path[0]]->params.p(1), nwp.y, nwp.x);
							break;
			case 9: nwp.x = 185; nwp.y = 194; break;
			case 10: back_path.push_back(1);
							back_path.push_back(0);
							_backtracking = true;
							srt.current_node()->cartesian_global_to_cell(srt.node()[back_path[0]]->params.p(0), srt.node()[back_path[0]]->params.p(1), nwp.y, nwp.x);
							break;
			default: break;
		}
		wp_counter++;
		
		
		
		double x, y;
		srt.current_node()->cell_to_cartesian_global(nwp.y, nwp.x, x, y);
		behavior("Goto")->set_option_double("goal_x", "uri_uav::GotoTask", x);
		behavior("Goto")->set_option_double("goal_y", "uri_uav::GotoTask", y);
		
		if (_backtracking){
			std::cout << "NEW BACKTRACKING " << nwp << " " << x << " " << y << std::endl;
		}
		else {
			std::cout << "NEW WAYPOINT " << nwp << " " << x << " " << y << std::endl;
		}
		
		
		
		srt.current_node()->print_stats();
		
		// show the current node
		cv::namedWindow( "current node", cv::WINDOW_AUTOSIZE );// Create a window for display.
		cv::Mat b;
		int scale = 1;
		srt.current_node()->grid_map_color(b,scale);
		
		// draw waypoint, center, path
		cv::Scalar dark_red(0.0, 0.0, 120.0);
		cv::Scalar red(0.0, 0.0, 255.0);
		cv::circle(b, nwp, 5, red, 2);
		cv::circle(b, srt.current_node()->params.center_cell, 5, dark_red, 2);
		cv::line(b, srt.current_node()->params.center_cell, nwp, red, 1);
		
		
		cv::imshow( "current node", b );                   // Show our image inside it.
		cv::waitKey(50);
		
		std::cout << "aaaa0" << std::endl;
		if (_savedata){
		std::cout << "aaaa1" << std::endl;
			for (int i=0; i<srt.node().size(); i++){
		std::cout << "aaaa2 " << i << std::endl;
				if (srt.current_node()->id() != i){
					cv::Mat im;
					int scale = 1;
					srt.node()[i]->grid_map_color(im,scale);
		std::cout << "aaaa3" << std::endl;
					
					std::stringstream namefile;
					namefile << "/home/paolos/exploration_data/node_" << i << "_" <<wp_counter << ".png";
					cv::imwrite( namefile.str().c_str(), im);
		std::cout << "aaaa4" << std::endl;
				}
				else{
		std::cout << "aaaa5" << std::endl;
					std::stringstream namefile;
		std::cout << "aaaa5.1" << std::endl;
					namefile << "/home/paolos/exploration_data/node_"<< i << "_" <<wp_counter << ".png";
		std::cout << "aaaa5.2" << std::endl;
					cv::imwrite( namefile.str().c_str(), b);
		std::cout << "aaaa6" << std::endl;
				}
			}
		}
		
		
		
// 		srt.node()[0]->show_grid_map_color(1,50, "aaa");
		_next_active_behavior = behavior("Goto");
// 			_ls_ready = true;
		_ls_ready = false;
	}
	
	return uri::Continue;
	
}


void SRTExploration::get_mandatory_resources(ResourceVector &res){
	
	std::string lint("uri_base::SharedMemory<uri_bridge::LaserScan>");
	ls = (uri_base::SharedMemory<uri_bridge::LaserScan>*)res.get_resource_ptr(lint);
	
}


};


