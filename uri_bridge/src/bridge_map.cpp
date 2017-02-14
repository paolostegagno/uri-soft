#include <iostream>
#include <vector>

#include "ros/ros.h"
#include <uri_bridge/bridge_map.hpp>

namespace uri_bridge{

	GridMap::GridMap(double csx, double csy, double msx, double msy){
		
		cell_size_x = csx;
		cell_size_y = csy;
		
		map_size_x  = msx;
		map_size_y  = msy;
		
		number_cells_x = 2*(int)msx/csx + 1;
		number_cells_y = 2*(int)msy/csy + 1;
		
		center_cell_x = number_cells_x/2;
		center_cell_y = number_cells_y/2;
		
		*(this->dataptr()) = Mat::zeros(number_cells_x, number_cells_y, CV_8UC1);
		frontier_mask = Mat::zeros(number_cells_x, number_cells_y, CV_8UC1);
		obstacle_mask = Mat::zeros(number_cells_x, number_cells_y, CV_8UC1);
		free_plus_safe_mask = Mat::zeros(number_cells_x, number_cells_y, CV_8UC1);
		safe_mask = Mat::zeros(number_cells_x, number_cells_y, CV_8UC1);
		free_mask = Mat::zeros(number_cells_x, number_cells_y, CV_8UC1);
	}
  
  
	void GridMap::print_stats(){
		std::cout << " cell size: " << this->cell_size_x << ',' << this->cell_size_y << std::endl;
		std::cout << " map size: " << this->map_size_x  << ',' << this->map_size_y  << std::endl;
		std::cout << " number cells: " << this->number_cells_x  << ',' << this->number_cells_y << std::endl;
		std::cout << " center cell: " << this->center_cell_x  << ',' << this->center_cell_y << std::endl;
	}
	
	
	void GridMap::coordinates_to_cell(double x, double y, int &a, int &b){
		a = (int)std::round(x/cell_size_x) + center_cell_x;
		b = (int)std::round(y/cell_size_y) + center_cell_y;
	}
	
	void GridMap::cell_to_coordinates(int a, int b, double &x, double &y){
		x = (a-center_cell_x)*cell_size_x;
		y = (b-center_cell_y)*cell_size_y;
	}
	
	
	void GridMap::import_scan(sensor_msgs::LaserScan &ls, double safety_clearance, double gap_as_frontier_distance){
		
		int num_readings = ls.ranges.size();
		double angle = ls.angle_min;
		int x_cell_p, y_cell_p;
		double x_p, y_p;
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
			coordinates_to_cell(x,y,x_cell,y_cell); // translate cartesian coordinates into cell indexes
			if (x_cell < number_cells_x && y_cell < number_cells_y){
				
				if (i>0){ // if two consecutive points are not in adiacent cells, then the cells in the middle must be filled either frontier or obstacle
					int dx_cell = x_cell - x_cell_p;
					int dy_cell = y_cell - y_cell_p;
					double dx, dy;
					
					dx = (x-x_p);
					dy = (y-y_p);
					double lenght = sqrt( dx*dx + dy*dy);
					if (range >= ls.range_max-0.1 && range_p >= ls.range_max-0.1) {
						cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), FRONTIER, 2);
					}
					else if (lenght>gap_as_frontier_distance) {
						cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), cv::Scalar(FRONTIER), 2);
					}
					else{
						cv::line(*(this->dataptr()), cv::Point(y_cell,x_cell), cv::Point(y_cell_p,x_cell_p), cv::Scalar(OBSTACLE), 2);
						if (range >= ls.range_max-0.1) {
							this->dataptr()->at<uchar>(x_cell,y_cell) = FRONTIER;
						}
						if (range_p >= ls.range_max-0.1) {
							this->dataptr()->at<uchar>(x_cell_p,y_cell_p) = FRONTIER;
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
		int safe_cells_band = (int)(safety_clearance/std::min(cell_size_x, cell_size_y));
		int alt_safe_cells_band = (int)(safety_clearance/std::max(cell_size_x, cell_size_y));
		
		// create mask containing only obstacle points (value 255)
		cv::inRange(*(this->dataptr()), OBSTACLE, OBSTACLE, obstacle_mask);
		
		// create mask containing only frontier points (value 255)
		cv::inRange(*(this->dataptr()), FRONTIER, FRONTIER, frontier_mask);
		
		/**/cv::namedWindow( "test", cv::WINDOW_AUTOSIZE );// Create a window for display.
		
		/**/cv::imshow("test", *(this->dataptr()));
		/**/cv::waitKey(0);

		
		free_plus_safe_mask = frontier_mask + obstacle_mask; // create mask with all frontier and obstacle points
		
		/**/cv::imshow("test", frontier_mask);
		/**/cv::waitKey(0);
		/**/cv::imshow("test", obstacle_mask);
		/**/cv::waitKey(0);
		/**/cv::imshow("test", free_plus_safe_mask);
		/**/cv::waitKey(0);
		
		// create a mask including both free and safe areas, i.e., all areas inside the 
		cv::floodFill(free_plus_safe_mask, cv::Point(0,0), FULL_PIXEL_VALUE, (cv::Rect*)0, cv::Scalar(), ZERO_PIXEL_VALUE); // put all points external to the boundary to FULL_PIXEL_VALUE
		free_plus_safe_mask = FULL_PIXEL_VALUE - free_plus_safe_mask;
		
		/**/cv::imshow("test", free_plus_safe_mask);
		/**/cv::waitKey(0);
		
		// erodes the free area in order to obtain the most internal portion
		cv::erode(free_plus_safe_mask,safe_mask,cv::Mat(),cv::Point(-1,-1), safe_cells_band );
		
		/**/cv::imshow("test", safe_mask);
		/**/cv::waitKey(0);
		
		free_mask = free_plus_safe_mask - safe_mask;
		
		this->dataptr()->setTo(FREE, free_mask);
		this->dataptr()->setTo(SAFE, safe_mask);
		
		/**/cv::imshow("test", free_mask);
		/**/cv::waitKey(0);
		
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

	
	
	void GridMap::show_grid_map(int scale, int wait_time, std::string window_name){
		cv::namedWindow( window_name, cv::WINDOW_AUTOSIZE );// Create a window for display.
		
		cv::Mat a(*(this->dataptr())),b,c;
		
		b = Mat::zeros(scale*number_cells_x, scale*number_cells_y, CV_8UC1);
		for (int i=0; i<number_cells_x; i++){
			for (int j=0; j<number_cells_y; j++){
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
	
	
	
	
	void GridMap::show_grid_map_color(int scale, int wait_time, std::string window_name){
		cv::namedWindow( window_name, cv::WINDOW_AUTOSIZE );// Create a window for display.
		
		cv::Mat a(*(this->dataptr())),b,c;
		
		b = Mat::zeros(scale*number_cells_x, scale*number_cells_y, CV_8UC3);
		for (int i=0; i<number_cells_x; i++){
			for (int j=0; j<number_cells_y; j++){
				uchar val = this->dataptr()->at<uchar>(i,j);
				for (int k=0; k<scale; k++){
					for (int h=0; h<scale; h++){
						cv::Vec3b color;
						switch (val){
							case UNKNOWN: 			color[0] = 255; color[1] = 255; color[2] = 255; 			b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color; break;
							case OBSTACLE: 				color[0] = 0; color[1] = 0; color[2] = 0;		b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case FRONTIER: 					color[0] = 0; color[1] = 255; color[2] = 0;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case FREE: 							color[0] = 100; color[1] = 255; color[2] = 255;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case SAFE_NEAR_FRONTIER: color[0] = 60; color[1] = 255; color[2] = 255;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							case SAFE: 							color[0] = 255; color[1] = 180; color[2] = 100;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
							default: 								color[0] = 0; color[1] = 0; color[2] = 0;	b.at<cv::Vec3b>(scale*i+k,scale*j+h) = color;  break;
						}
					}
				}
			}
		}
		
		cv::imshow( window_name, b );                   // Show our image inside it.
		if (wait_time>=0)
			cv::waitKey(wait_time);                         // Wait for a keystroke in the window
	}
	
	
	
	
	void GridMap::find_blobs(const cv::Mat &label_image, std::vector < Frontier > &blobs)
	{
		blobs.clear();
		
		cv::namedWindow("ttt", cv::WINDOW_AUTOSIZE);

		// Fill the label_image with the blobs
		// 0  - background
		// 1  - unlabelled foreground
		// 2+ - labelled foreground

// 		cv::Mat label_image;
// 		binary.convertTo(label_image, CV_32SC1);
		
// 		std::cout << label_image << std::endl;
		
		cv::imshow("ttt", label_image);
		cv::waitKey(0);

		int label_count = 2; // starts at 2 because 0,1 are used already

		std::cout << "c " << label_image.rows << " " << label_image.cols << std::endl;

		for(int y=0; y < label_image.rows; y++) {
			uchar *row = (uchar*)label_image.ptr(y);
			for(int x=0; x < label_image.cols; x++) {
				if(row[x] != 255) {
					continue;
				}
				
				std::cout << "y " << y << " x " << x  << " row[x] " << (int)row[x] << std::endl;


				cv::Rect rect;
// 								std::cout << label_image << std::endl;

				cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);
				
				std::cout << label_count << std::endl;
// 				std::cout << rect << std::endl;
		
		cv::imshow("ttt", label_image);
		cv::waitKey(0);
		
				Frontier blob;
				
				cv::Mat a;
				
				label_image(rect).copyTo(a);
				std::cout << a << std::endl;

				for(int i=rect.y; i < (rect.y+rect.height); i++) {
					uchar *row2 = (uchar*)label_image.ptr(i);
					for(int j=rect.x; j < (rect.x+rect.width); j++) {
						if(row2[j] != label_count) {
							continue;
						}

						blob.push_back(cv::Point2i(j,i));
					}
				}

				blobs.push_back(blob);

				label_count++;
			}
		}

	}

	
	
	
	
	
	
	
	

};


