
#include <Eigen/Geometry>


#ifndef __ANGLE_CONVERSION_HPP__
#define __ANGLE_CONVERSION_HPP__

namespace uri_base{

	/// @brief Converts Eigen::Quaterniond to roll, pitch and yaw angles
	/// @param[in] &q Input Quaternion 
	/// @param[out] &roll output roll
	/// @param[out] &pitch output pitch
	/// @param[out] &yaw output yaw
	void quaternion_to_rpy(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
	
	/// @brief Extracts yaw from Eigen::Quaterniond
	/// @param[in] &q Input Quaternion 
	/// @return yaw.
	double quaternion_to_yaw(const Eigen::Quaterniond& q);
	
	/// @brief Extracts yaw from Eigen::Quaterniond
	/// @param[in] &q Input Quaternion 
	/// @param[out] &yaw output yaw
	void quaternion_to_yaw(const Eigen::Quaterniond& q, double& yaw);
	
	/// @brief Converts roll pitc yaw into Eigen::Quaterniond
	/// @param[in] &pitch Input pitch
	/// @param[in] &roll Input roll
	/// @param[in] &yaw Input yaw
	/// @return The converted Eigen::Quaternion
	Eigen::Quaterniond rpy_to_quaternion(double roll, double pitch, double yaw);
	


}


#endif