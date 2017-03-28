
#include <vector>
#include <iostream>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#ifndef __SRTNODE_HPP__
#define __SRTNODE_HPP__

// using namespace uri;



namespace uri_bridge{
	
#define UNKNOWN 0
#define OBSTACLE 30
#define FRONTIER 120
#define UNINTERESTING_FRONTIER 100
#define WAYPOINT 150
#define FREE 180
#define SAFE_NEAR_FRONTIER 220
#define SAFE 255
	
#define FULL_PIXEL_VALUE 255
#define ZERO_PIXEL_VALUE 0
	
	
	
// 	typedef int CellValue;
	
	
	
	/// \brief A collection of points creating a connected piece of frontier
	class Frontier : public std::vector<cv::Point2i> {
		
	public:
		/// @brief Default Constructor
		Frontier(){
			this->clear();
		}
		
	};
	
	
	
	/// \brief All the parameters of a SRTNode collected in a single struct
	struct SRTNodeParams{
		
		double cell_size_x;
		double cell_size_y;
		
		double map_size_x;
		double map_size_y;
		
		int number_cells_x;
		int number_cells_y;
		
		cv::Point2i center_cell;
		
		double safety_distance_from_obstacles_or_frontier;
		int safe_cells_band;
		
		double gap_as_frontier_threshold;
		
		Eigen::Vector2d p;
		
		double border_x_pos;
		double border_x_neg;
		double border_y_pos;
		double border_y_neg;
		
		
		/// @brief Complete constructor.
		/// @param[in] _p position of the center cell of the map in the world frame
		/// @param[in] csx cell size along the x axis, default value 0.05 m
		/// @param[in] csy cell size along the y axis, default value 0.05 m
		/// @param[in] msx map size along the x axis, default value 5.00 m
		/// @param[in] msy map size along the y axis, default value 5.00 m
		/// @param[in] sd safety distance from any obstacle or frontier, default value 0.50 m
		/// @param[in] gft threshold to consider what is between two obstacles as a frontier (obstacle if lower), default value 0.50 m
		SRTNodeParams (Eigen::Vector2d _p, double csx = 0.05, double csy = 0.05, double msx = 5.00, double msy = 5.00, double sd = 0.5, double gft = 0.5);
		
		/// @brief Copy constructor.
		/// @param[in] &other Copied SRTNodeParams.
		SRTNodeParams (const SRTNodeParams& other);
		
		/// @brief Copy operator.
		/// @param[in] &other Copied SRTNodeParams.
		SRTNodeParams& operator=(const SRTNodeParams& other);
		
	};
	
	
	/// \brief This class represents a grid map
	/// \details The map is represented through a cv::Mat
	class SRTNode : public cv::Mat{
		
		std::vector<Frontier> safe_frontiers;
		
		cv::Mat frontier_mask;
		cv::Mat uninteresting_frontier_mask;
		cv::Mat obstacle_mask;
		cv::Mat free_plus_safe_mask;
		cv::Mat safe_mask;
		cv::Mat free_mask;
		
		SRTNode* _parent;
		std::vector<SRTNode*> _child;
		int _level;
		
		int _id;
		
	public:
		
		std::vector<SRTNode*> path_to_here;
		SRTNode* expanded_me;
		
	public:
		
		
		
		
		
		SRTNodeParams params;
		
		/// @brief complete constructor.
		/// @param[in] parptr pointer to the parent node
		/// @param[in] gmp set of parameters
		SRTNode(SRTNode* parptr, SRTNodeParams& gmp);
		
		/// @brief Copy Constructor.
		/// @param[in] other copied SRTNode
		SRTNode(const SRTNode& other);
		
		/// @brief Copy operator.
		/// @param[in] other copied SRTNode
		SRTNode& operator=(const SRTNode& other);
		
		
		/// @brief Returns \b this.
		SRTNode* ptr(){
			return this;
		}
		
		/// @brief Returns a cv::Mat* pointer at \b this.
		cv::Mat* dataptr() const{
			return (cv::Mat*)this;
		}
		
		/// @brief overload operator << for ostream.
		friend std::ostream& operator<<(std::ostream& os, const SRTNode& g);  
		
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
		void cartesian_local_to_cell(double x, double y, int &a, int &b);
		
		/// @brief translates a b cell indexes into x y coordinates w.r.t. the center of the map
		/// @param[in] a map row
		/// @param[in] b map column
		/// @param[out] &x x coordinate
		/// @param[out] &y y coordinate
		void cell_to_cartesian_local(int a, int b, double &x, double &y);
		
		
		/// @brief translates x y coordinates in world frame into cell indexes
		/// @param[in] x x coordinate
		/// @param[in] y y coordinate
		/// @param[out] &a map row
		/// @param[out] &b map column
		void cartesian_global_to_cell(double x, double y, int &a, int &b);
		
		/// @brief translates a b cell indexes into x y coordinates in world frame
		/// @param[in] a map row
		/// @param[in] b map column
		/// @param[out] &x x coordinate
		/// @param[out] &y y coordinate
		void cell_to_cartesian_global(int a, int b, double &x, double &y);
		
		/// @brief translates x y coordinates w.r.t. the center of the map into cell indexes
		/// @param[in] x x coordinate
		/// @param[in] y y coordinate
		/// @param[out] &a map row
		/// @param[out] &b map column
		void cartesian_local_to_cartesian_global(double x, double y, double &xg, double &yg);
		
		/// @brief check if a point in the global frame is within the mapped area
		/// @param[in] x x coordinate [m]
		/// @param[in] y y coordinate [m]
		/// @return \b true if it is within the mapped area, \b false otherwise 
		bool cartesian_global_in_map(double x, double y);
		
		/// @brief create the map based on the provided scan.
		/// @param[in] ls a laser scanner in the format of a ROS message.
		/// @details this methods performs the following actions:\n
		/// 1) all scan points whose range is equal to ls.range_max are projected on the map and set as FRONTIER \n
		/// 2) all scan points whose range is less than ls.range_max are projected on the map and set as OBSTACLE \n
		/// 3) all cells in gaps between two FRONTIER cells originating from consecutive scan points are set as FRONTIER following a straight line. \n
		/// 4) all cells in gaps between two OBSTACLE cells or an OBSTACLE and a FRONTIER cell originating from consecutive scan points are: \n
		///    - set as FRONTIER following a straight line if the gap between the two points is larger than \b gap_as_frontier_threshold \n
		///    - set as OBSTACLE following a straight line if the gap between the two points is less than \b gap_as_frontier_threshold \n
		/// 5) all map cells contained into the FRONTIER/OBSTACLE border and with distance
		/// from any OBSTACLE or FRONTIER cell less than than safety_clearance are set as FREE \n
		/// 6) all map cells contained into the FRONTIER/OBSTACLE border and with distance
		/// from any OBSTACLE or FRONTIER cell greater than safety_clearance are set as SAFE
		void import_scan(sensor_msgs::LaserScan &ls);
		
		
		/// @brief Shows the gridmap in an OpenCV window.
		/// @param[in] scale the map is scaled by this factor when displayed (i.e., each cell is represented by a square of scale X scale pixels) (default 1)
		/// @param[in] wait_time time in ms the image is displayed before closing the window. If wait_time=0 (default) the window stays open until a key is hit on it. If  wait_time<0 the window closes immediately.
		/// @param[in] window_name name of the OpenCV window (default = "SRTNode").
		void show_grid_map(int scale=1, int wait_time = 0, std::string window_name="SRTNode");
		
		/// @brief Shows the gridmap in an OpenCV window.
		/// @param[in] scale the map is scaled by this factor when displayed (i.e., each cell is represented by a square of scale X scale pixels) (default 1)
		/// @param[in] wait_time time in ms the image is displayed before closing the window. If wait_time=0 (default) the window stays open until a key is hit on it. If  wait_time<0 the window closes immediately.
		/// @param[in] window_name name of the OpenCV window (default = "SRTNode").
		void show_grid_map_color(int scale=1, int wait_time = 0, std::string window_name="SRTNode");
		
		/// @brief Provides a colored version of the gridmap.
		/// @param[out] &b The image is stored in here.
		/// @param[in] scale the map is scaled by this factor when displayed (i.e., each cell is represented by a square of scale X scale pixels) (default 1)
		void grid_map_color(cv::Mat& b, int scale=1);
		
		/// @brief Extracts frontier blobs
		void find_blobs(const cv::Mat &binary, std::vector < Frontier > &blobs);
		
		/// @brief Selects next waypoint based on frontiers, obstacles, ...
		/// @return cv::Point2i containg the cell coordinates of the selected waypoint
		cv::Point2i select_next_waypoint();
		
		/// @brief Create a new child node
		/// @param[in] _pos The coordinates of the center cel in world frame
		/// @param[in] scan The scan used to ceate the map.
		/// @return a pointer to the new SRTNode
		SRTNode* create_new_child(SRTNodeParams &gmp, sensor_msgs::LaserScan scan);
		
		
		/// @brief update the map using another node  
		/// @param[in] other other node
		void update_node(SRTNode* other);
		
		/// @brief Check if the node has Frontiers
		/// @return \b true if the node has Frontiers, \b false otherwise.
		bool has_frontiers(){
			if (safe_frontiers.size() > 0){
				return true;
			}
			return false;
		}
		
		/// @brief set the level of the node
		/// @details level(first_node) = 0, level(child) = level(parent)+1.
		/// @param[in] lv level.
		void set_level(int lv){
			_level = lv;
		}
		
		/// @brief get the level of the node
		/// @details level(first_node) = 0, level(child) = level(parent)+1.
		/// @return the level of the node.
		int level(){
			return _level;
		}
		
		/// @brief set the id of the node
		/// @param[in] i id.
		void set_id(int i){
			_id = i;
		}
		
		/// @brief get the id of the node
		/// @return the id of the node.
		int id(){
			return _id;
		}
		
		
		/// @brief get the parent of the node
		/// @return a pointer to the parent of the node.
		SRTNode* parent(){return _parent;}
		
		/// @brief get the std::vector of childs of the node
		/// @return a pointer to the std::vector of childs of the node.
		std::vector<SRTNode*>& child(){
			return _child;
		}
		
		
	};
	
	
	
// 	std::ostream& operator<<(std::ostream& os, const SRTNode& g)  
// 	{  
// 		os << "cell size: " << g.params.cell_size_x << ',' << g.params.cell_size_y ;
// 		os << " map size: " << g.params.map_size_x  << ',' << g.params.map_size_y  ;
// 		os << " number cells: " << g.params.number_cells_x  << ',' << g.params.number_cells_y;
// 		os << " center cell: " << g.params.center_cell ;
// 		os << std::endl ;
// 		os << *(g.dataptr()) << std::endl;
// // 		for (int i=0; i < g.number_cells_x ; i++){
// // 			for (int j=0; j < g.number_cells_y ; j++){
// // // 					std::cout << (*this)(i,j);
// // 			}
// // 		}
// 		return os;
// 	}
	
	

	
	
};


#endif