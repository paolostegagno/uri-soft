cmake_minimum_required(VERSION 2.8.3)
project(uri_uav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mav_msgs
  uri_core
  uri_base
  pluginlib
  cmake_modules
)


find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uri_uav
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

## Declare a C++ library
add_library(uri_uav
	src/iris_interface.cpp
	src/tasks/takeoff.cpp
	src/tasks/land.cpp
	src/tasks/hover.cpp
	src/tasks/goto.cpp
	src/tasks/trajectory_controller.cpp
	src/tasks/yawratecontroller.cpp
	src/tasks/bcontrollerexample.cpp)
target_link_libraries(uri_uav ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
message(WARNING ${EIGEN_LIBRARIES})


