

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <sstream>
#include <iostream>
#include <string>

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"




#include <pluginlib/class_list_macros.h>
#include <uri_core/resource.hpp>

#include <Eigen/Geometry>




#ifndef __IRIS_INTERFACE_HPP__
#define __IRIS_INTERFACE_HPP__

using namespace uri;

namespace uri_uav {

/// @brief Interface for pixhawk-based URI copters
/// @details This class uses some services provided by mavros in order
/// to provide commands to the uav.
class IrisInterface: public Resource{
  
  
  // main ROS nodehandle
//   ros::NodeHandle _n;
  
	// here all publishers which writes on topics read by mavros
	ros::Publisher _pub_setpoint_velocity_cmd_vel;
	
	// here all publishers which writes on topics read by mavros
	ros::Publisher _pub_setpoint_attitude_ext_cmd_vel;
	ros::Publisher _pub_setpoint_attitude_ext_attitude;
	ros::Publisher _pub_setpoint_attitude_ext_att_throttle;
	
	// here all publishers which writes on topics read by mavros
	ros::Publisher _pub_setpoint_raw_local;
	ros::Publisher _pub_setpoint_raw_global;
	ros::Publisher _pub_setpoint_raw_attitude;
	ros::Publisher _pub_setpoint_raw_ext_local_ext;
	ros::Publisher _pub_setpoint_raw_ext_global_ext;
	ros::Publisher _pub_setpoint_raw_ext_attitude_ext;
	
	ros::Publisher _pub_setpoint_position_local;
	ros::Publisher _pub_mavros_rc_override;
	ros::Publisher _pub_setpoint_accel_accel;
	

	// here all subscribers which reads topics published by mavros and theyr respective callback function
	ros::Subscriber _sub_local_position_pose;
	void _local_position_pose_CB(const geometry_msgs::PoseStamped::ConstPtr& msg);
	bool _local_position_pose_received;
	
	// here all subscribers which reads topics published by mavros and theyr respective callback function
	ros::Subscriber _sub_state;
	void _state_CB(const mavros_msgs::State::ConstPtr& msg);

	// here all service clients which calls services from mavros with their respective messages
	ros::ServiceClient _srv_set_mode;
	mavros_msgs::SetMode _msg_set_mode;

	ros::ServiceClient _srv_cmd_arming;
	mavros_msgs::CommandBool _msg_cmd_arming;

	ros::ServiceClient _srv_cmd_takeoff;
	mavros_msgs::CommandTOL _msg_cmd_takeoff;

	ros::ServiceClient _srv_cmd_land;
	mavros_msgs::CommandTOL _msg_cmd_land;


	void _init();

	Eigen::Vector3d _position;
	Eigen::Quaterniond _orientation;
	bool _connected;
	bool _armed;
	bool _guided;
	std::string _mode;


	public:
		/// @brief Standard constructor
		/// @details Only takes as input parameter a ros::NodeHandle.
		/// @param[in] &n A valid ros::NodeHandle.
		IrisInterface(ros::NodeHandle &n);
		
		/// @brief Do-nothing constructor
		/// @details This method does not build anything. To be followed by initialize(ros::NodeHandle &n).
		IrisInterface();

	// 		/// @brief Standard constructor
	//     /// @details Only takes as input parameter a ros::NodeHandle.
	//     /// @param[in] &n A valid ros::NodeHandle.
	//     void initialize(ros::NodeHandle &n);
		
		/// @brief Set operating mode
		/// @details This method sends a request to set the operating mode. Possible modes are "guided", "..."
		/// @param[in] mode string containing the name of the desired mode.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool setMode(std::string mode);
		
		/// @brief Arm the propellers
		/// @details This method sends a request to arm the propellers.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool armThrottle();

		/// @brief Disarm the propellers
		/// @details This method sends a request to disarm the propellers.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool disarmThrottle();
		
		/// @brief Takeoff
		/// @details This method sends a request to takeoff.
		/// @param[in] altitude desired takeoff relative altitude.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool takeoff(double altitude);
		
		/// @brief Takeoff
		/// @details This method sends a request to takeoff.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool land();
		
		/// @brief Check receiving of local position
		/// @details This method tells whether a pose over the topic
		/// "/mavros/local_position/pose" has ever been received.
		/// @return \b true if a _local_position_pose_ has ever been received, \b false otherwise.
		bool local_position_pose_received(){
			return _local_position_pose_received;
		}
		
		/// @brief Send commanded velocity
		/// @details this method sends a setpoint_velocity cmd_vel message to the UAV
		void commandVelocity(double x, double y, double z);
		
		/// @brief Send trajectory command
		/// @details this method sends a setpoint position, velocity, acceleration, yaw and yaw rate
		void commandTrajectory(Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a, double yaw, double yawrate);
		
		/// @brief Send trajectory command
		/// @details this method sends a setpoint position, velocity, acceleration, yaw and yaw rate
// 		void commandAttitude(Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a, double yaw, double yawrate);
		
		
	void commandAngularVelocity(double rr, double pr, double yr){
		geometry_msgs::TwistStamped msg;
		msg.twist.angular.x=rr;
		msg.twist.angular.x=pr;
		msg.twist.angular.x=yr;
		
		_pub_setpoint_attitude_ext_cmd_vel.publish<geometry_msgs::TwistStamped>(msg);
	}
	
	void commandAttitude(Eigen::Quaterniond qu){
		geometry_msgs::PoseStamped msg;
		msg.pose.orientation.x = qu.x();
		msg.pose.orientation.y = qu.y();
		msg.pose.orientation.z = qu.z();
		msg.pose.orientation.w = qu.w();
		std::cout << "here " << std::endl;
		_pub_setpoint_attitude_ext_attitude.publish<geometry_msgs::PoseStamped>(msg);
	}
	
	void commandAttitudeThrottle(){
		std_msgs::Float64 msg;
		_pub_setpoint_attitude_ext_att_throttle.publish<std_msgs::Float64>(msg);
	}
		
		
		
		/// @brief Get current position.
		inline Eigen::Vector3d& position(){
			return _position;
		}
		
		/// @brief Get current orientation.
		inline Eigen::Quaterniond& orientation(){
			return _orientation;
		}
		
		/// @brief Get current orientation.
		inline double yaw(){
			Eigen::Vector3d euler = _orientation.toRotationMatrix().eulerAngles(2, 1, 0);
			return euler[0];
		}
		
		/// @brief Get connected bool.
		inline bool connected(){
			return _connected;
		}
		
		/// @brief Get armed bool.
		inline bool armed(){
			return _armed;
		}
		
		/// @brief Get guided bool.
		inline bool guided(){
			return _guided;
		}
		
		/// @brief Get current mode.
		inline std::string& mode(){
			return _mode;
		}



};


}

  PLUGINLIB_EXPORT_CLASS(uri_uav::IrisInterface, uri::Resource)



#endif






