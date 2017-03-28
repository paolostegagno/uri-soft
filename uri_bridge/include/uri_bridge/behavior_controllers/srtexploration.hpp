
#include <pluginlib/class_list_macros.h>
#include <uri_core/behavior_controller.hpp>



#include <uri_bridge/resources/shared_memory.hpp>

#include <uri_bridge/SRT.hpp>




#ifndef __SRTEXPLORATION_HPP__
#define __SRTEXPLORATION_HPP__

using namespace uri;

namespace uri_bridge{
	
	class SRTExploration: public BehaviorController{
		
	private:
		
		uri_base::SharedMemory<uri_bridge::LaserScan>* ls;
		
		SRT srt;
		bool _ls_ready;
		
		int wp_counter;
		
		std::vector<int> back_path;
		bool _backtracking;
		bool _savedata;
		
		
		
		virtual TaskOutput __run();
		
		virtual void _initialize(){}
		
		virtual void _activate(){}
		
		virtual void _deactivate(){}
		
		
	public:
		
		SRTExploration();
		
		void get_mandatory_resources(ResourceVector& res);
		
	};

  PLUGINLIB_EXPORT_CLASS(uri_bridge::SRTExploration, uri::BehaviorController)
	
	
};


#endif