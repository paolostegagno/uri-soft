
#include <vector>
#include <iostream>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "SRTNode.hpp"

#ifndef __SRT_HPP__
#define __SRT_HPP__



namespace uri_bridge{
	
	
	/// @brief A map for srt exploration.
	class SRT{
		
		std::vector<SRTNode*> _node;
		
		SRTNode* _current;
		
	public:
		
		/// \brief Constructor
		SRT();
		
		
		/// \brief Add a node to the exploration tree
		/// \param[in] &gmp parameters of the SRTNode of this node
		/// \param[in] ls Laser scanner message to generate the gridmap;
		void add_node(SRTNodeParams &gmp, sensor_msgs::LaserScan ls);
		
		
		/// @brief Compute the next waypoint over the tree, and the path to reach it
		cv::Point2i compute_next_waypoint();
		
		
		/// @brief this method finds the closest node with Frontier to the given node
		/// @brief The distance is calculated as distance on the tree
		/// @param[in] start_node pointer to the start node
		/// @param[out] cnwf in this variable the pointer to the selected node is stored. If no node as frontier in the whole tree, \b cnwf is set at \b nullptr.
		void find_closest_node_with_frontiers(SRTNode* start_node, SRTNode* &cnwf);
		
		
		void print_current_node(){
			_current->show_grid_map_color(2, 0);
		}
		
		std::vector<SRTNode*>& node(){
			return _node;
		}
		
		/// @brief Access method to the current node.
		/// @details The current node is the last visited node. It is also the last created node, unless the algorithm is in backtracking.
		/// @return a pointer to the current node.
		SRTNode* current_node(){
			return _current;
		}
		
		/// @brief Set the current node.
		/// @param[in] id id of the new current node.
		void set_current_node(int id) {
			_current = _node[id];
		}
		
		
	};
	
	
	
};


#endif