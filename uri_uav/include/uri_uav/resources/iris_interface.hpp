

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <sstream>
#include <iostream>
#include <string>

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/BatteryStatus.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/AttitudeTarget.h"




#include <pluginlib/class_list_macros.h>
#include <uri_core/resource.hpp>

#include <uri_base/angle_conversion.hpp>

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
	
	ros::Timer _timer_interpolate_poses;
	ros::Timer _timer_monitor;
	void _timer_interpolate_poses_CB(const ros::TimerEvent& event);
	double delta_t;
	ros::Time start_t;
	double last_elapsed;


	// here all subscribers which reads topics published by mavros and their respective callback function
	ros::Subscriber _sub_local_position_pose;
	void _local_position_pose_CB(const geometry_msgs::PoseStamped::ConstPtr& msg);
	bool _local_position_pose_received;
	
	ros::Subscriber _sub_local_position_velocity;
	void _local_position_velocity_CB(const geometry_msgs::TwistStamped::ConstPtr& msg);
	
	// here all subscribers which reads topics published by mavros and their respective callback function
	ros::Subscriber _sub_state;
	void _state_CB(const mavros_msgs::State::ConstPtr& msg);

	// here all subscribers which reads topics published by mavros and their respective callback function
	ros::Subscriber _sub_battery;
	void _battery_CB(const mavros_msgs::BatteryStatus::ConstPtr& msg);


	// here all service clients which calls services from mavros with their respective messages
	ros::ServiceClient _srv_set_mode;
	mavros_msgs::SetMode _msg_set_mode;
	
	ros::ServiceClient _srv_set_stream_rate;
	mavros_msgs::StreamRate _msg_set_stream_rate;

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
	
	Eigen::Vector3d _velocity_lin;
	Eigen::Vector3d _velocity_ang;

	double _battery_voltage;
	double _battery_current;
	double _battery_remaining;
	
	
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
		
		/// @brief Set stream rate for all the topics
		/// @details This method sends a request to set the stream rate of all topics to the passed value
		/// @param[in] rate desired rate.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool setRate(unsigned int rate);
		
		/// @brief Set stream rate for all the raw sensor topics
		/// @details This method sends a request to set the stream rate of all topics to the passed value
		/// @param[in] rate desired rate.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool setRateRawSensors(unsigned int rate);

		/// @brief Set stream rate for all the raw sensor topics
		/// @details This method sends a request to set the stream rate of all topics to the passed value
		/// @param[in] rate desired rate.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool setRateExtendedStatus(unsigned int rate);

		/// @brief Set stream rate for all the raw sensor topics
		/// @details This method sends a request to set the stream rate of all topics to the passed value
		/// @param[in] rate desired rate.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool setRateControllerRaw(unsigned int rate);
		
		/// @brief Set stream rate for all the raw sensor topics
		/// @details This method sends a request to set the stream rate of all topics to the passed value
		/// @param[in] rate desired rate.
		/// @return \b true if the request is fullfilled, \b false otherwise.
		bool setRatePosition(unsigned int rate);
		
		
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
		
		/// @brief Send yaw rate command
		/// @details this method sends a setpoint_raw attitude message to the UAV, setting only the yawrate
		void commandYawrate(double yr);

		
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
		
		
		/// @brief commands the attitude and thrust of the UAV in guided or stabilize mode
		void commandAttitudeThrottle(float thr, Eigen::Quaterniond ori){
			
			
			mavros_msgs::AttitudeTarget _msg_setpoint_raw_attitude;
			
// 			char a = -58;    
// 			std::bitset<8> x(a);
// 			std::cout << x;
			
// 			short c = -315;
// 			std::bitset<16> y(c);
// 			std::cout << y;
			
// 			uint8_t mask = (1 << 7) | (1 << 6) | (1 << 1) | (1 << 0);
			
// 			std::cout << "!!! " << mask << std::endl;
			
			_msg_setpoint_raw_attitude.body_rate.x = 0.0;
			_msg_setpoint_raw_attitude.body_rate.y = 0.0;
			_msg_setpoint_raw_attitude.body_rate.z = 0.0;
			_msg_setpoint_raw_attitude.orientation.x = ori.x();
			_msg_setpoint_raw_attitude.orientation.y = ori.y();
			_msg_setpoint_raw_attitude.orientation.z = ori.z();
			_msg_setpoint_raw_attitude.orientation.w = ori.w();
			_msg_setpoint_raw_attitude.thrust = thr;
			_msg_setpoint_raw_attitude.type_mask = (uint8_t)7;
			
			_pub_setpoint_raw_attitude.publish<mavros_msgs::AttitudeTarget>(_msg_setpoint_raw_attitude);
		}
		
		/// @brief Get current position.
		inline Eigen::Vector3d& position(){
			return _position;
		}
		
		/// @brief Get current orientation.
		inline Eigen::Quaterniond& orientation(){
			return _orientation;
		}
		
		/// @brief Get current linear velocity.
		inline Eigen::Vector3d& velocity_linear(){
			return _velocity_lin;
		}
		
		/// @brief Get current angular velocity.
		inline Eigen::Vector3d& velocity_angular(){
			return _velocity_ang;
		}
		
		/// @brief Get current orientation.
		inline double yaw(){
			return uri_base::quaternion_to_yaw(_orientation);
// 			Eigen::Vector3d euler = _orientation.toRotationMatrix().eulerAngles(2, 1, 0);
// 			return euler[0];
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

		/// @brief Get current battery voltage.
		inline double battery_voltage(){
			return _battery_voltage;
		}

		/// @brief Get current battery current.
		inline double battery_current(){
			return _battery_current;
		}

		/// @brief Get current battery remaining.
		inline double battery_remaining(){
			return _battery_remaining;
		}

		/// @brief Get start time. 
		inline ros::Time start_time(){
			return start_t;
		}

};


}

  PLUGINLIB_EXPORT_CLASS(uri_uav::IrisInterface, uri::Resource)



#endif






