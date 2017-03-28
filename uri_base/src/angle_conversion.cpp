
#include <uri_base/angle_conversion.hpp>


namespace uri_base{

	void quaternion_to_rpy(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
	{
		double ysqr = q.y() * q.y();

		// roll (x-axis rotation)
		double t0 = +2.0f * (q.w() * q.x() + q.y() * q.z());
		double t1 = +1.0f - 2.0f * (q.x() * q.x() + ysqr);
		roll = std::atan2(t0, t1);

		// pitch (y-axis rotation)
		double t2 = +2.0f * (q.w() * q.y() - q.z() * q.x());
		t2 = t2 > 1.0f ? 1.0f : t2;
		t2 = t2 < -1.0f ? -1.0f : t2;
		pitch = std::asin(t2);

		// yaw (z-axis rotation)
		double t3 = +2.0f * (q.w() * q.z() + q.x() *q.y());
		double t4 = +1.0f - 2.0f * (ysqr + q.z() * q.z());  
		yaw = std::atan2(t3, t4);
	}


	double quaternion_to_yaw(const Eigen::Quaterniond& q)
	{
		double ysqr = q.y() * q.y();
		
		// yaw (z-axis rotation)
		double t3 = +2.0f * (q.w() * q.z() + q.x() *q.y());
		double t4 = +1.0f - 2.0f * (ysqr + q.z() * q.z());  
		return std::atan2(t3, t4);
	}

	void quaternion_to_yaw(const Eigen::Quaterniond& q, double& yaw)
	{
		double ysqr = q.y() * q.y();
		
		// yaw (z-axis rotation)
		double t3 = +2.0f * (q.w() * q.z() + q.x() *q.y());
		double t4 = +1.0f - 2.0f * (ysqr + q.z() * q.z());  
		yaw = std::atan2(t3, t4);
	}


	Eigen::Quaterniond rpy_to_quaternion(double roll, double pitch, double yaw)
	{
		Eigen::Quaterniond q;
		double t0 = std::cos(yaw * 0.5f);
		double t1 = std::sin(yaw * 0.5f);
		double t2 = std::cos(roll * 0.5f);
		double t3 = std::sin(roll * 0.5f);
		double t4 = std::cos(pitch * 0.5f);
		double t5 = std::sin(pitch * 0.5f);

		q.w() = t0 * t2 * t4 + t1 * t3 * t5;
		q.x() = t0 * t3 * t4 - t1 * t2 * t5;
		q.y() = t0 * t2 * t5 + t1 * t3 * t4;
		q.z() = t1 * t2 * t4 - t0 * t3 * t5;
		return q;
	}
	
	
	Eigen::Matrix3d rpy_to_rot(double roll, double pitch, double yaw){
		Eigen::Matrix3d R;
		R(0,0) = cos(pitch)*cos(yaw);	R(0,1) = cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw);	R(0,2) = sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch);
		R(1,0) = cos(pitch)*sin(yaw);	R(1,1) = cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw);	R(1,2) = cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll);
		R(2,0) = -sin(pitch);					R(2,1) = cos(pitch)*sin(roll);																R(2,2) = cos(pitch)*cos(roll);
		
		return R;
	}




}


