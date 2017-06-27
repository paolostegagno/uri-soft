
#include <uri_base/shared_memory.hpp>

// #include "uri_base/trajectory.hpp"
#include <uri_base/trajectory.hpp>



namespace uri_base{


PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<uri_base::Trajectory>, uri::Resource);

PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<uri_base::Heading>, uri::Resource);

PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<uri_base::Pose>, uri::Resource);

PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<uri_base::TwoByNMatrix>, uri::Resource);

PLUGINLIB_EXPORT_CLASS(uri_base::SharedMemory<uri_base::String>, uri::Resource);

}; // end namespace




