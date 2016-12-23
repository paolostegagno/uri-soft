
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



}


