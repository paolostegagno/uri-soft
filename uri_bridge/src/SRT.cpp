#include <uri_bridge/SRT.hpp>

namespace uri_bridge{
	
	
	SRT::SRT(){
		_node.clear();
		_current = nullptr;
	}
	
	
	
	
	void SRT::add_node(SRTNodeParams &gmp, sensor_msgs::LaserScan ls){
		
		// first, we address the creation of the first node
		if (_current == nullptr) {
// 			std::cout << "0AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
			_current = new SRTNode(nullptr, gmp);
			_current->import_scan(ls);
			_node.push_back(_current);
			_current->set_id(0);
// 			std::cout << "1AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
		}
		// here we create a child node to _current, use it to update all nodes in the tree and set it as _current
		else {
// 			std::cout << "2AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
			SRTNode* new_child = _current->create_new_child(gmp, ls);
			for (int i=0; i < _node.size(); i++){
				_node[i]->update_node(new_child);
			}
			_current = new_child;
			_node.push_back(_current);
			_current->set_id(_node.size()-1);
// 			std::cout << "3AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
		}
	}
	
	
	
	
	
	
	// compute the next waypoint over the tree, and the path to reach it
	cv::Point2i SRT::compute_next_waypoint(){
		
		// if _current has Frontiers, then explore locally
		if (_current->has_frontiers()){
			return _current->select_next_waypoint();
		}
		
		// otherwise, we have to find the closest node with Frontiers
		_current->path_to_here.clear();
		_current->expanded_me = nullptr;
		
		SRTNode* closest_node_with_frontiers;
		find_closest_node_with_frontiers(_current, closest_node_with_frontiers);
		
		// if one node with Frontiers exists, then compute new waypoint there
		if (closest_node_with_frontiers != nullptr) {
			return closest_node_with_frontiers->select_next_waypoint();
		}
		
		// otherwise, the exploration is terminated and we should go back home
		else return _node[0]->params.center_cell;
	}
	
	
	
	
	
	
	void SRT::find_closest_node_with_frontiers(SRTNode* start_node, SRTNode* &cnwf){
		
		std::vector<SRTNode*> search_set;
		search_set.push_back(start_node);
		
		while(true) {
			std::vector<SRTNode*> searchable_nodes;
			for (int j=0; j<search_set.size(); j++) {
				for (int i=0; i<search_set[j]->child().size()+1; i++){
					SRTNode* temp;
					if(i < search_set[j]->child().size()){
						temp = search_set[j]->child()[i];
					}
					else {
						temp = search_set[j]->parent();
					}
					// ignore the node if it is either null, or is the one that expanded the current search node
					if (temp == nullptr or temp == search_set[j]->expanded_me){
						continue;
					}
					temp->expanded_me = search_set[j];
					temp->path_to_here = search_set[j]->path_to_here;
					temp->path_to_here.push_back(search_set[j]);
					searchable_nodes.push_back(temp);
				}
			}
			
			// if there are no searchable nodes, there are no nodes with frontier to expand.
			// the search must be terminated with no output (UAV will be homing)
			if (searchable_nodes.size() == 0){
				cnwf = nullptr;
				return;
			}
			
			// among searchable nodes, check which has frontiers.
			std::vector<SRTNode*> frontier_nodes;
			for (int i=0; i<searchable_nodes.size(); i++){
				if (searchable_nodes[i]->has_frontiers()){
					frontier_nodes.push_back(searchable_nodes[i]);
				}
			}
			// if any node among the searchable has frontier, pick one of them as next node to visit.
			if (frontier_nodes.size()>0){
				cnwf = frontier_nodes[0];
				return;
				// in this case we have to pick one
				// TODO implement smart picking!!! now it takes the first one
			}
			// if no node has frontier, do another iteration of the algorithm by expanding all the searchable nodes
			else {
				search_set = searchable_nodes;
			}
		}
	}
	
	
	
};


