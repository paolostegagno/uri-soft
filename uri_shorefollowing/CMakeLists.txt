cmake_minimum_required(VERSION 2.8.3)
project(uri_shorefollowing)

set(CMAKE_CXX_FLAGS "-g -std=c++11")
# set(CMAKE_C '-std=c++11')
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
  uri_uav
  uri_sensors
  pluginlib
  cmake_modules
)


find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uri_shorefollowing
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
add_library(uri_shorefollowing
		src/tasks/circling.cpp
		src/tasks/shorefollowing.cpp
		src/tasks/shorefollowingpaolo.cpp
		src/behavior_controllers/circlingexperiment.cpp
		src/behavior_controllers/shorefollowingexperiment.cpp
		src/behavior_controllers/shoredetectionexperiment.cpp
		src/tasks/fixedspeeddesiredheadingcontroller.cpp
	)
# target_link_libraries(uri_shorefollowing ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})


