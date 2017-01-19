
#include <vector>
#include <iostream>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#ifndef __BRIDGE_MAP_HPP__
#define __BRIDGE_MAP_HPP__

// using namespace uri;



namespace uri_bridge{
	
#define UNKNOWN 0
#define OBSTACLE 30
#define FRONTIER 120
#define FREE 180
#define SAFE_NEAR_FRONTIER 220
#define SAFE 255
	
#define FULL_PIXEL_VALUE 255
#define ZERO_PIXEL_VALUE 0
	
// 	/// @brief 
// 	enum CellValue : int{
// 		UNKNOWN = 0,
// 		FREE = 1,
// 		FRONTIER = 2,
// 		OBSTACLE = 3
// 	};
	
	
	typedef int CellValue;
	
// 	template<typename T>
// 	std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
// 	{
// 		return stream << static_cast<typename std::underlying_type<T>::type>(e);
// 	}
	
	
	class Frontier : public std::vector<cv::Point2i> {
		
	public:
		/// @brief Default Constructor
		Frontier(){
			this->clear();
		}
		
// 		cv::Point2i baricenter(){
// 			cv::Point2i
// 			
// 			for (int i=0; i<
// 			
// 			
// 			return cv::Point2i
// 		}
		
		
		
	};
	
	
	
// 	class GridMap : public Eigen::Matrix<CellValue,Eigen::Dynamic,Eigen::Dynamic>{
	class GridMap : public cv::Mat{
		
		
		double cell_size_x;
		double cell_size_y;
		
		double map_size_x;
		double map_size_y;
		
		int number_cells_x;
		int number_cells_y;
		
		int center_cell_x;
		int center_cell_y;
		
		std::vector<Frontier> safe_frontiers;
		
		cv::Mat frontier_mask;
		cv::Mat obstacle_mask;
		cv::Mat free_plus_safe_mask;
		cv::Mat safe_mask;
		cv::Mat free_mask;
		
	public:
		
		/// @brief complete constructor.
		/// @param[in] csx cell size along the x axis, default value 0.05 m
		/// @param[in] csy cell size along the y axis, default value 0.05 m
		/// @param[in] msx map size along the x axis, default value 5.00 m
		/// @param[in] msy map size along the y axis, default value 5.00 m
		GridMap(double csx = 0.05, double csy = 0.05, double msx = 5.00, double msy = 5.00);
		
		cv::Mat* dataptr() const{
			return (cv::Mat*)this;
		}
		
		/// @brief overload operator << for ostream.
		friend std::ostream& operator<<(std::ostream& os, const GridMap& g);  
		
		/// @brief prints the map on screan as if it was a cv::Mat (actually it is).
		void print(){
			std::cout << *(this->dataptr()) << std::endl;
		}
		
		/// @brief prints some statistics about the map.
		void print_stats();
		
		/// @brief translates x y coordinates w.r.t. the center of the map into cell indexes
		/// @param[in] x x coordinate
		/// @param[in] y y coordinate
		/// @param[out] &a map row
		/// @param[out] &b map column
		void coordinates_to_cell(double x, double y, int &a, int &b);
		
		/// @brief translates a b cell indexes into x y coordinates w.r.t. the center of the map
		/// @param[in] a map row
		/// @param[in] b map column
		/// @param[out] &x x coordinate
		/// @param[out] &y y coordinate
		void cell_to_coordinates(int a, int b, double &x, double &y);
		
		/// @brief create the map based on the provided scan.
		/// @param[in] ls a laser scanner in the format of a ROS message.
		/// @details this methods performs the following actions:\n
		/// 1) all scan points whose range is equal to ls.range_max are projected on the map and set as FRONTIER \n
		/// 2) all scan points whose range is less than ls.range_max are projected on the map and set as OBSTACLE \n
		/// 3) all cells in gaps between two FRONTIER cells originating from consecutive scan points are set as FRONTIER following a straight line. \n
		/// 4) all cells in gaps between two OBSTACLE cells or an OBSTACLE and a FRONTIER cell originating from consecutive scan points are: \n
		///    - set as FRONTIER following a straight line if the gap between the two points is larger than \b gap_as_frontier_distance \n
		///    - set as OBSTACLE following a straight line if the gap between the two points is less than \b gap_as_frontier_distance \n
		/// 5) all map cells contained into the FRONTIER/OBSTACLE border and with distance
		/// from any OBSTACLE or FRONTIER cell less than than safety_clearance are set as FREE \n
		/// 6) all map cells contained into the FRONTIER/OBSTACLE border and with distance
		/// from any OBSTACLE or FRONTIER cell greater than safety_clearance are set as SAFE
		void import_scan(sensor_msgs::LaserScan &ls, double safety_clearance=0.301, double gap_as_frontier_distance=0.5);
		
		
		/// @brief Shows the gridmap in an OpenCV window.
		/// @param[in] scale the map is scaled by this factor when displayed (i.e., each cell is represented by a square of scale X scale pixels) (default 1)
		/// @param[in] wait_time time in ms the image is displayed before closing the window. If wait_time=0 (default) the window stays open until a key is hit on it. If  wait_time<0 the window closes immediately.
		/// @param[in] window_name name of the OpenCV window (default = "GridMap").
		void show_grid_map(int scale=1, int wait_time = 0, std::string window_name="GridMap");
		
		
		void find_blobs(const cv::Mat &binary, std::vector < Frontier > &blobs);
		
	};
	
	
	
	std::ostream& operator<<(std::ostream& os, const GridMap& g)  
	{  
		os << "cell size: " << g.cell_size_x << ',' << g.cell_size_y ;
		os << " map size: " << g.map_size_x  << ',' << g.map_size_y  ;
		os << " number cells: " << g.number_cells_x  << ',' << g.number_cells_y;
		os << " center cell: " << g.center_cell_x  << ',' << g.center_cell_y ;
		os << std::endl ;
		os << *(g.dataptr()) << std::endl;
// 		for (int i=0; i < g.number_cells_x ; i++){
// 			for (int j=0; j < g.number_cells_y ; j++){
// // 					std::cout << (*this)(i,j);
// 			}
// 		}
		return os;
	}

	
	
	
	
	/// @brief A node of the srt map.
	class SRTNode{
		
		double x;
		double y;
		
		GridMap map;
		
	public:
		/// @brief Null constructor.
		SRTNode(){}
		
		/// @brief Position Constructor.
		SRTNode(int a, int b){
			x=a;
			y=b;
		}
		
		/// @brief destructor.
		~SRTNode(){}
		
		/// @brief Copy operator.
		SRTNode& operator=(const SRTNode& other)
		{
			if (this != &other) {
			}
			return *this;
		}
		
	};
	
	

	
	/// @brief A map for srt exploration.
	class SRTMap{
		
		std::vector<SRTNode> node;
		
	public:
		
		void add_node(){
			
		}
		
		
		
	};
	
	
	
	
	
	
	
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