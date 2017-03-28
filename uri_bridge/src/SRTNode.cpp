#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_bridge/SRTNode.hpp>

namespace uri_bridge{
	
	SRTNodeParams::SRTNodeParams(Eigen::Vector2d _p, double csx, double csy, double msx, double msy, double sd, double gft){
		
		p = _p;
		
		cell_size_x = csx;
		cell_size_y = csy;
		
		map_size_x  = msx;
		map_size_y  = msy;
		
		number_cells_x = 2*(int)(msx/csx) + 1;
		number_cells_y = 2*(int)(msy/csy) + 1;
		
		center_cell.x = number_cells_x/2;
		center_cell.y = number_cells_y/2;
		
		safety_distance_from_obstacles_or_frontier = sd;
		safe_cells_band = (int)(safety_distance_from_obstacles_or_frontier/std::min(cell_size_x, cell_size_y));
		
		gap_as_frontier_threshold = gft;
		
		border_x_pos = p(0) + map_size_x;
		border_x_neg = p(0) - map_size_x;
		border_y_pos = p(1) + map_size_y;
		border_y_neg = p(1) - map_size_y;
	}
	
	
	SRTNodeParams::SRTNodeParams (const SRTNodeParams& other){
		
		p = other.p;
		
		cell_size_x = other.cell_size_x;
		cell_size_y = other.cell_size_y;
		
		map_size_x = other.map_size_x;
		map_size_y = other.map_size_y;
		
		number_cells_x = other.number_cells_x;
		number_cells_y = other.number_cells_y;
		
		center_cell = other.center_cell;
		
		safety_distance_from_obstacles_or_frontier = other.safety_distance_from_obstacles_or_frontier;
		safe_cells_band = other.safe_cells_band;
		
		gap_as_frontier_threshold = other.gap_as_frontier_threshold;
		
		border_x_pos = other.border_x_pos;
		border_x_neg = other.border_x_neg;
		border_y_pos = other.border_y_pos;
		border_y_neg = other.border_y_neg;
	}
	
	
	SRTNodeParams& SRTNodeParams::operator=(const SRTNodeParams& other){
		if (this != &other) {
			
			p = other.p;
			
			cell_size_x = other.cell_size_x;
			cell_size_y = other.cell_size_y;
			
			map_size_x = other.map_size_x;
			map_size_y = other.map_size_y;
			
			number_cells_x = other.number_cells_x;
			number_cells_y = other.number_cells_y;
			
			center_cell = other.center_cell;
			
			safety_distance_from_obstacles_or_frontier = other.safety_distance_from_obstacles_or_frontier;
			safe_cells_band = other.safe_cells_band;
			
			gap_as_frontier_threshold = other.gap_as_frontier_threshold;
			
			border_x_pos = other.border_x_pos;
			border_x_neg = other.border_x_neg;
			border_y_pos = other.border_y_pos;
			border_y_neg = other.border_y_neg;
		}
		return *this;
	}
	
	
	SRTNode::SRTNode(SRTNode* parptr, SRTNodeParams& gmp):params(gmp){
		
		_parent = parptr;
		_child.clear();
		_level = 0;
		
		*(this->dataptr()) = Mat::zeros(params.number_cells_x, params.number_cells_y, CV_8UC1);
		frontier_mask = Mat::zeros(params.number_cells_x, params.number_cells_y, CV_8UC1);
		uninteresting_frontier_mask = Mat::zeros(params.number_cells_x, params.number_cells_y, CV_8UC1);
		obstacle_mask = Mat::zeros(params.number_cells_x, params.number_cells_y, CV_8UC1);
		free_plus_safe_mask = Mat::zeros(params.number_cells_x, params.number_cells_y, CV_8UC1);
		safe_mask = Mat::zeros(params.number_cells_x, params.number_cells_y, CV_8UC1);
		free_mask = Mat::zeros(params.number_cells_x, params.number_cells_y, CV_8UC1);
		
		srand (time(NULL));
		
	}
	
	
	
	
	SRTNode::SRTNode(const SRTNode& other) : params(other.params)
	{
		_parent = other._parent;
		_child = other._child;
		_level = other._level;
		
		safe_frontiers  = other.safe_frontiers;
		
		frontier_mask = other.frontier_mask;
		uninteresting_frontier_mask = other.uninteresting_frontier_mask;
		obstacle_mask = other.obstacle_mask;
		free_plus_safe_mask = other.free_plus_safe_mask;
		safe_mask = other.safe_mask;
		free_mask = other.free_mask;
		*(this->dataptr()) = *(other.dataptr());
	}
	
	
	SRTNode& SRTNode::operator=(const SRTNode& other)
	{
		if (this != &other) {
			params = other.params;
			
			_parent = other._parent;
			_child = other._child;
			_level = other._level;
			
			safe_frontiers  = other.safe_frontiers;
			
			frontier_mask = other.frontier_mask;
			uninteresting_frontier_mask = other.uninteresting_frontier_mask;
			obstacle_mask = other.obstacle_mask;
			free_plus_safe_mask = other.free_plus_safe_mask;
			safe_mask = other.safe_mask;
			free_mask = other.free_mask;
			*(this->dataptr()) = *(other.dataptr());
		}
		return *this;
	}
	
	
	
  
  
	void SRTNode::print_stats(){
		std::cout << " cell size: " << this->params.cell_size_x << ',' << this->params.cell_size_y << std::endl;
		std::cout << " map size: " << 2*this->params.map_size_x  << ',' << 2*this->params.map_size_y  << std::endl;
		std::cout << " number cells: " << this->params.number_cells_x  << ',' << this->params.number_cells_y << std::endl;
		std::cout << " center cell: " << this->params.center_cell << std::endl;
		std::cout << " global center: " << this->params.p(0) << " " << this->params.p(1) << std::endl;
		std::cout << " vertices: [" << params.border_x_neg << " " << params.border_y_neg
							<< "] [" << params.border_x_pos << " " << params.border_y_neg
							<< "] [" << params.border_x_neg << " " << params.border_y_pos 
							<< "] [" << params.border_x_pos << " " << params.border_y_pos << std::endl;
	}
	
	
	void SRTNode::cartesian_local_to_cell(double x, double y, int &a, int &b){
		a = (int)std::round(x/params.cell_size_x) + params.center_cell.x;
		b = (int)std::round(y/params.cell_size_y) + params.center_cell.y;
	}
	
	
	void SRTNode::cell_to_cartesian_local(int a, int b, double &x, double &y){
		x = (a-params.center_cell.x)*params.cell_size_x;
		y = (b-params.center_cell.y)*params.cell_size_y;
	}
	
	
	void SRTNode::cartesian_global_to_cell(double x, double y, int &a, int &b){
		cartesian_local_to_cell(x-params.p(0), y-params.p(1), a, b);
	}
	
	void SRTNode::cell_to_cartesian_global(int a, int b, double &x, double &y){
		cell_to_cartesian_local(a, b, x, y);
		x += params.p(0);
		y += params.p(1);
// 		double temp = y;
// 		y = x;
// 		x = temp;
	}
	
	void SRTNode::cartesian_local_to_cartesian_global(double x, double y, double &xg, double &yg){
		xg = x + params.p(0);
		yg = y + params.p(1);
	}
	
	
	bool SRTNode::cartesian_global_in_map(double x, double y){
		if (x>params.border_x_neg && x<params.border_x_pos && y>params.border_y_neg && y<params.border_y_pos){
			return true;
		}
		return false;
// 		if (x>params.bou
	}
	
	
	void SRTNode::import_scan(sensor_msgs::LaserScan &ls){
		
		int num_readings = ls.ranges.size();
		double angle = ls.angle_min;
		int x_cell_p, y_cell_p;
		double x_p, y_p;
// 		bool prev_interesting = true;
		double x_f, y_f;
		int x_cell, y_cell;
		double x, y;
		double range, range_p;
		
		for(unsigned int i = 0; i < num_readings+1; ++i){
			if (i < num_readings) { // every point but the last point
				range = ls.ranges[i]; // traslate polar coordinates into cartesian coordinates
				if (range <0.2){
					angle += ls.angle_increment;
					continue;
// 					range = (ls.ranges[i-1] + ls.ranges[i+1])/2;
				}
				x = range * std::cos(angle);
				y = range * std::sin(angle);
			}
			else { // the last time the very first point is selected again
				x = x_f;
				y = y_f;
				range = ls.ranges[0];
			}
			cartesian_local_to_cell(x,y,x_cell,y_cell); // translate cartesian coordinates into cell indexes
			if (x_cell < params.number_cells_x && y_cell < params.number_cells_y){
				
				if (i>0){ // if two consecutive points are not in adiacent cells, then the cells in the middle must be filled either frontier or obstacle
					int dx_cell = x_cell - x_cell_p;
					int dy_cell = y_cell - y_cell_p;
					double dx, dy;
					
					dx = (x-x_p);
					dy = (y-y_p);
					double lenght = sqrt( dx*dx + dy*dy);
					
					double xg, yg, xg_p, yg_p;
					cartesian_local_to_cartesian_global(x, y, xg, yg);
					cartesian_local_to_cartesian_global(x_p, y_p, xg_p, yg_p);
					bool in_range = true;
					if ( (yg<1.0 or yg>18.0) and (yg_p<1.0 or yg_p>18.0)) {
						in_range = false;
					}
					if ( (xg<-3.5 or xg>20.0) and (xg_p<-3.5 or xg_p>20.0)) {
						in_range = false;
					}
						
					if (range >= ls.range_max-0.1 && range_p >= ls.range_max-0.1) {
						if (in_range) {
							cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), cv::Scalar(FRONTIER), 2);
						}
						else {
							cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), UNINTERESTING_FRONTIER, 2);
						}
					}
					else if (lenght>params.gap_as_frontier_threshold) {
						if (in_range) {
							cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), cv::Scalar(FRONTIER), 2);
						}
						else {
							cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), UNINTERESTING_FRONTIER, 2);
						}
					}
					else{
						cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), cv::Scalar(OBSTACLE), 2);
						if (range >= ls.range_max-0.1) {
							if (in_range) {
								this->dataptr()->at<uchar>(x_cell,y_cell) = FRONTIER;
							}
							else {
								this->dataptr()->at<uchar>(x_cell,y_cell) = UNINTERESTING_FRONTIER;
							}
						}
						if (range_p >= ls.range_max-0.1) {
							if (in_range) {
								this->dataptr()->at<uchar>(x_cell_p,y_cell_p) = FRONTIER;
							}
							else {
								this->dataptr()->at<uchar>(x_cell_p,y_cell_p) = UNINTERESTING_FRONTIER;
							}
						}
					}
				}
			}
			if (i == 0){
				x_f = x;
				y_f = y;
			}
			x_cell_p = x_cell;
			y_cell_p = y_cell;
			x_p = x;
			y_p = y;
			range_p = range;
			angle += ls.angle_increment;
		}

// 		Mat freearea_mask, safearea_mask, g, freenonsafearea, safearea, h;

		
		// create mask containing only obstacle points (value 255)
		cv::inRange(*(this->dataptr()), OBSTACLE, OBSTACLE, obstacle_mask);
		
		// create mask containing only frontier points (value 255)
		cv::inRange(*(this->dataptr()), FRONTIER, FRONTIER, frontier_mask);
		
		// create mask containing only uninteresting frontier points (value 255)
		cv::inRange(*(this->dataptr()), UNINTERESTING_FRONTIER, UNINTERESTING_FRONTIER, uninteresting_frontier_mask);
		
		
// 		/**/cv::namedWindow( "test", cv::WINDOW_AUTOSIZE );// Create a window for display.
		
// 		/**/cv::imshow("test", *(this->dataptr()));
// 		/**/cv::waitKey(0);

		
		free_plus_safe_mask = uninteresting_frontier_mask + frontier_mask + obstacle_mask; // create mask with all frontier and obstacle points
		
// 		/**/cv::imshow("test", frontier_mask);
// 		/**/cv::waitKey(0);
// 		/**/cv::imshow("test", obstacle_mask);
// 		/**/cv::waitKey(0);
// 		/**/cv::imshow("test", free_plus_safe_mask);
// 		/**/cv::waitKey(0);
		
		// create a mask including both free and safe areas, i.e., all areas inside the 
		cv::floodFill(free_plus_safe_mask, cv::Point(0,0), FULL_PIXEL_VALUE, (cv::Rect*)0, cv::Scalar(), ZERO_PIXEL_VALUE); // put all points external to the boundary to FULL_PIXEL_VALUE
		free_plus_safe_mask = FULL_PIXEL_VALUE - free_plus_safe_mask;
		
// 		/**/cv::imshow("test", free_plus_safe_mask);
// 		/**/cv::waitKey(0);
		
		// erodes the free area in order to obtain the most internal portion
		cv::erode(free_plus_safe_mask,safe_mask,cv::Mat(),cv::Point(-1,-1), params.safe_cells_band );
		
// 		/**/cv::imshow("test", safe_mask);
// 		/**/cv::waitKey(0);
		
		free_mask = free_plus_safe_mask - safe_mask;
		
		this->dataptr()->setTo(FREE, free_mask);
		this->dataptr()->setTo(SAFE, safe_mask);
		
// 		/**/cv::imshow("test", free_mask);
// 		/**/cv::waitKey(0);
		
		// here we extract connected components of the frontier
		cv::Mat binary;
		frontier_mask.copyTo(binary);
		
		find_blobs(binary, safe_frontiers);
// 		std::cout << "b" << std::endl;
// 		std::cout << safe_frontiers.size() << std::endl;
// 		for (int i=0; i<safe_frontiers.size();i++){
// 			std::cout << "    " << safe_frontiers[i].size() << std::endl;
// 		}
		
	}

	
	
	void SRTNode::show_grid_map(int scale, int wait_time, std::string window_name){
		cv::namedWindow( window_name, cv::WINDOW_AUTOSIZE );// Create a window for display.
		
		cv::Mat a(*(this->dataptr())),b,c;
		
		b = Mat::zeros(scale*params.number_cells_x, scale*params.number_cells_y, CV_8UC1);
		for (int i=0; i<params.number_cells_x; i++){
			for (int j=0; j<params.number_cells_y; j++){
				uchar val = this->dataptr()->at<uchar>(i,j);
				for (int k=0; k<scale; k++){
					for (int h=0; h<scale; h++){
						b.at<uchar>(scale*i+k,scale*j+h) = val;
					}
				}
			}
		}
		
		cv::imshow( window_name, b );                   // Show our image inside it.
		if (wait_time>=0)
			cv::waitKey(wait_time);                         // Wait for a keystroke in the window
	}
	
	
	
	
	void SRTNode::show_grid_map_color(int scale, int wait_time, std::string window_name){
		cv::namedWindow( window_name, cv::WINDOW_AUTOSIZE );// Create a window for display.
		
		cv::Mat b;
		grid_map_color(b, scale);
		
		cv::imshow( window_name, b );                   // Show our image inside it.
		if (wait_time>=0)
			cv::waitKey(wait_time);                         // Wait for a keystroke in the window
	}
	
	
	
	
	void SRTNode::grid_map_color(cv::Mat& b, int scale){
		
		b = Mat::zeros(scale*params.number_cells_x, scale*params.number_cells_y, CV_8UC3);
		for (int i=0; i<params.number_cells_x; i++){
			for (int j=0; j<params.number_cells_y; j++){
				uchar val = this->dataptr()->at<uchar>(i,j);
				for (int k=0; k<scale; k++){
					for (int h=0; h<scale; h++){
						cv::Vec3b color;
						switch (val){
							case UNKNOWN: 							color[0] = 255; color[1] = 255; color[2] = 255;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color; break;
							case OBSTACLE: 							color[0] = 0; 	color[1] = 0; 	color[2] = 0;		b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case UNINTERESTING_FRONTIER:color[0] = 255; color[1] = 0; 	color[2] = 0;		b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case FRONTIER: 							color[0] = 0; 	color[1] = 255; color[2] = 0;		b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case FREE: 									color[0] = 100; color[1] = 255; color[2] = 255;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case SAFE_NEAR_FRONTIER:		color[0] = 60; 	color[1] = 255; color[2] = 255;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case SAFE: 									color[0] = 255; color[1] = 180; color[2] = 100;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case WAYPOINT: 							color[0] = 0; 	color[1] = 0; 	color[2] = 255;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							default: 										color[0] = 0; 	color[1] = 0; 	color[2] = 0;		b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
						}
					}
				}
			}
		}
	}

	
	
	void SRTNode::find_blobs(const cv::Mat &label_image, std::vector < Frontier > &blobs)
	{
		blobs.clear();
		
		
		int label_count = 2; // starts at 2 because 0,1 are used already
		
		for(int y=0; y < label_image.rows; y++) {
			uchar *row = (uchar*)label_image.ptr(y);
			for(int x=0; x < label_image.cols; x++) {
				if(row[x] != 255) {
					continue;
				}
				
				cv::Rect rect;
				cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);
		
				Frontier blob;
				blob.clear();
				cv::Mat a;
				
				label_image(rect).copyTo(a);

				for(int i=rect.y; i < (rect.y+rect.height); i++) {
					uchar *row2 = (uchar*)label_image.ptr(i);
					for(int j=rect.x; j < (rect.x+rect.width); j++) {
						if(row2[j] != label_count) {
							continue;
						}
						blob.push_back(cv::Point2i(j,i));
						if (blob.size() > 500){
							blobs.push_back(blob);
							blob.clear();
						}
					}
				}
				// if 
				if (blob.size()>10){
					blobs.push_back(blob);
				}
				label_count++;
			}
		}

	}

	
	
	
	
	cv::Point2i SRTNode::select_next_waypoint(){
		
		cv::inRange(*(this->dataptr()), FRONTIER, FRONTIER, frontier_mask);
		
		// find and store all frontier points
		cv::Mat binary;
		frontier_mask.copyTo(binary);
		find_blobs(binary, safe_frontiers);
		
		
// 		long int num_frontier_cells = 0;
// 		for (int i=0; i<safe_frontiers.size(); i++){
// 			num_frontier_cells += safe_frontiers[i].size();
// 		}
		
// 		long int selected_cell = rand()%num_frontier_cells;
		
		
		int selected_blob = rand()%safe_frontiers.size();
		int selected_cell = safe_frontiers[selected_blob].size()/2;
		std::cout << safe_frontiers.size() << "  sf " << selected_blob << "   sb   " << selected_cell << std::endl;
		int i = selected_blob;
		
// 		safe_frontiers[i][selected_cell]
		
		
// 		long int counter = 0;
// 		int i;
// 		for (i=0; i<safe_frontiers.size(); i++){
// 			counter += safe_frontiers[i].size();
// 			if (counter > selected_cell){
// 				counter -= safe_frontiers[i].size();
// 				break;
// 			}
// 		}
// 		selected_cell -= counter;
		
// 		std::cout << " selected " << selected_cell << safe_frontiers[i][selected_cell] << std::endl;
// 		dataptr()->at<uchar>(safe_frontiers[i][selected_cell]) = WAYPOINT;
		int cell_value = dataptr()->at<uchar>(safe_frontiers[i][selected_cell]);
// 		std::cout << " cell value " << cell_value << std::endl;
		
		cv::Point2i end = safe_frontiers[i][selected_cell];
		cv::Point2d delta_p = end - params.center_cell;
// 			std::cout << " delta_p " << delta_p << std::endl;
		
		double max_delta = std::max(abs(delta_p.x), abs(delta_p.y));
		cv::Point2i next_waypoint = params.center_cell;
		cv::Point2i next_waypoint_1 = params.center_cell;
		cv::Point2i next_waypoint_2 = params.center_cell;
		cv::Point2i next_waypoint_3 = params.center_cell;
		cv::Point2i next_waypoint_4 = params.center_cell;
		cv::Point2i next_cell = params.center_cell;
		for (int j = 0; j < max_delta; j++){
			next_cell.x = (int)(delta_p.x*(double)(j)/max_delta) + params.center_cell.x;
			next_cell.y = (int)(delta_p.y*(double)(j)/max_delta) + params.center_cell.y;
// 			std::cout << " next_cell " << next_cell << " " << (int)dataptr()->at<uchar>(next_cell) << std::endl;
			if (dataptr()->at<uchar>(next_cell) == SAFE){
// 				dataptr()->at<uchar>(next_cell) = WAYPOINT;
				next_waypoint_4 = next_waypoint_3;
				next_waypoint_3 = next_waypoint_2;
				next_waypoint_2 = next_waypoint_1;
				next_waypoint_1 = next_waypoint;
				next_waypoint = next_cell;
			}
			else {
				break;
			}
		}
// 		cv::Point2i temp = next_waypoint - params.center_cell;
// 		cv::Point2i temp2(temp.y*3/4, temp.x*3/4);
// 		next_waypoint_4 = temp2 + params.center_cell;
		return next_waypoint_4;
	}
	
	
	
	
	
	SRTNode* SRTNode::create_new_child(SRTNodeParams &gmp, sensor_msgs::LaserScan scan){
		
		SRTNode* new_child = new SRTNode(this, gmp);
// 		std::cout << "c" << std::endl;
		
		new_child->import_scan(scan);
		new_child->set_level(_level+1);

// 		std::cout << "d" << std::endl;
		
		_child.push_back(new_child);
// 		std::cout << "e" << std::endl;
		
		return new_child;
	}
	
	
	void SRTNode::update_node(SRTNode* other){
		
		// with these conditions, we consider all cases in which the two maps does not intersect and return
		if (other->params.border_y_neg > params.border_y_pos){ return; }
		if (other->params.border_y_pos < params.border_y_neg){ return; }
		if (other->params.border_x_neg > params.border_x_pos){ return; }
		if (other->params.border_x_pos < params.border_x_neg){ return; }
		
		double x1, y1, x2, y2;
		// if we are here, the two maps intersect
		if (cartesian_global_in_map(other->params.border_x_neg, other->params.border_y_neg)){
// 			std::cout <<"aa " << other->params.border_x_neg << " " << other->params.border_y_neg << std::endl;
			x1 = other->params.border_x_neg;
			y1 = other->params.border_y_neg;
		}
		else if (cartesian_global_in_map(other->params.border_x_pos, other->params.border_y_neg)){
// 			std::cout <<"bb " << other->params.border_x_pos << " " << other->params.border_y_neg << std::endl;
			x1 = other->params.border_x_pos;
			y1 = other->params.border_y_neg;
		}
		else if (cartesian_global_in_map(other->params.border_x_neg, other->params.border_y_pos)){
// 			std::cout <<"cc " << other->params.border_x_neg << " " << other->params.border_y_pos << std::endl;
			x1 = other->params.border_x_neg;
			y1 = other->params.border_y_pos;
		}
		else if (cartesian_global_in_map(other->params.border_x_pos, other->params.border_y_pos)){
// 			std::cout <<"dd " << other->params.border_x_pos << " " << other->params.border_y_pos << std::endl;
			x1 = other->params.border_x_pos;
			y1 = other->params.border_y_pos;
		}
		
		if (other->cartesian_global_in_map(params.border_x_neg, params.border_y_neg)){
// 		std::cout <<"aaa " << params.border_x_neg << " " << params.border_y_neg << std::endl;
			x2 = params.border_x_neg;
			y2 = params.border_y_neg;
		}
		else if (other->cartesian_global_in_map(params.border_x_pos, params.border_y_neg)){
			x2 = params.border_x_pos;
			y2 = params.border_y_neg;
// 		std::cout <<"aab " << params.border_x_pos << " " << params.border_y_neg << std::endl;
		}
		else if (other->cartesian_global_in_map(params.border_x_neg, params.border_y_pos)){
			x2 = params.border_x_neg;
			y2 = params.border_y_pos;
// 		std::cout <<"aac " << params.border_x_neg << " " << params.border_y_pos << std::endl;
		}
		else if (other->cartesian_global_in_map(params.border_x_pos, params.border_y_pos)){
			x2 = params.border_x_pos;
			y2 = params.border_y_pos;
// 		std::cout <<"aad " << params.border_x_pos << " " << params.border_y_pos << std::endl;
		}
		
		double x_neg = std::min(x1, x2), x_pos = std::max(x1, x2);
		double y_neg = std::min(y1, y2), y_pos = std::max(y1, y2);
// 		std::cout << "["<< x_neg << " " << y_neg << "] [" << x_pos << " " << y_pos << "]" <<  std::endl;
		int a_neg1, b_neg1, a_pos1, b_pos1;
		int a_neg2, b_neg2, a_pos2, b_pos2;
		cartesian_global_to_cell(x_neg, y_neg, a_neg1, b_neg1);
		cartesian_global_to_cell(x_pos, y_pos, a_pos1, b_pos1);
		other->cartesian_global_to_cell(x_neg, y_neg, a_neg2, b_neg2);
		other->cartesian_global_to_cell(x_pos, y_pos, a_pos2, b_pos2);
		
		for (int i1 = a_neg1, i2 = a_neg2; i1 <= a_pos1 && i2 <= a_pos2; i1++, i2++){
			for (int j1 = b_neg1, j2 = b_neg2; j1 <= b_pos1 && j2 <= b_pos2; j1++, j2++){
				if (at<uchar>(i1,j1) == OBSTACLE) {
					other->at<uchar>(i2,j2) = OBSTACLE;
				}
				else if (other->at<uchar>(i2,j2) == OBSTACLE) {
					at<uchar>(i1,j1) = OBSTACLE;
				}
				else if (at<uchar>(i1,j1) == SAFE) {
					other->at<uchar>(i2,j2) = SAFE;
				}
				else if (other->at<uchar>(i2,j2) == SAFE) {
					at<uchar>(i1,j1) = SAFE;
				}
				else if (at<uchar>(i1,j1) == FREE) {
					other->at<uchar>(i2,j2) = FREE;
				}
				else if (other->at<uchar>(i2,j2) == FREE) {
					at<uchar>(i1,j1) = FREE;
				}
				else if (at<uchar>(i1,j1) == FRONTIER) {
					other->at<uchar>(i2,j2) = FRONTIER;
				}
				else if (other->at<uchar>(i2,j2) == FRONTIER) {
					at<uchar>(i1,j1) = FRONTIER;
				}
				else if (at<uchar>(i1,j1) == UNINTERESTING_FRONTIER) {
					other->at<uchar>(i2,j2) = UNINTERESTING_FRONTIER;
				}
				else if (other->at<uchar>(i2,j2) == UNINTERESTING_FRONTIER) {
					at<uchar>(i1,j1) = UNINTERESTING_FRONTIER;
				}
			}
		}
// 		for (int i = a_neg2; i <= a_pos2; i++){
// 			for (int j = b_neg2; j <= b_pos2; j++){
// 				other->at<uchar>(j,i) = other->at<uchar>(j,i) + 10;
// 			}
// 		}
// 		std::cout << "["<< a_neg1 << " " << b_neg1 << "] [" << a_pos1 << " " << b_pos1 << "]" <<  std::endl;
// 		std::cout << "["<< a_neg2 << " " << b_neg2 << "] [" << a_pos2 << " " << b_pos2 << "]" <<  std::endl;
// 		std::cout << " " << a_pos1 - a_neg1 << " " << b_pos1 - b_neg1 << std::endl;
// 		std::cout << " " << a_pos2 - a_neg2 << " " << b_pos2 - b_neg2 << std::endl;
		
	}
	
	
	
	
	

};


