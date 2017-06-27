cmake_minimum_required(VERSION 2.8.3)
project(uri_base)



# if(UNIX)
#     SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -Ofast -march=native")
# endif()


set(CMAKE_CXX_FLAGS "-g -std=c++11")
# set(CMAKE_C '-std=c++11')
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  uri_core
#   pluginlib
  cmake_modules
)


find_package(Eigen REQUIRED)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		uri_base
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
add_library(uri_base
	src/angle_conversion.cpp
	src/trajectory.cpp
	src/shared_memory.cpp
	src/keyboard.cpp
	)
target_link_libraries(uri_base ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
#message(WARNING ${EIGEN_LIBRARIES})


