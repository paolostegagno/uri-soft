cmake_minimum_required(VERSION 2.8.3)
project(uri_uav)

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
  uri_sensors
  cmake_modules
)


find_package(Eigen REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uri_uav uri_uav_resources
  CATKIN_DEPENDS roscpp rospy uri_core uri_base
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
add_library(uri_uav_resources
	src/resources/iris_interface.cpp
	)
	
## Add cmake target dependencies of the library
add_dependencies(uri_uav_resources ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(uri_uav_resources ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
# message(WARNING ${EIGEN_LIBRARIES})


## Declare a C++ library
add_library(uri_uav
	src/tasks/takeoff.cpp
	src/tasks/land.cpp
	src/tasks/hover.cpp
	src/tasks/goto.cpp
	src/tasks/trajectory_controller.cpp
	src/tasks/yawratecontroller.cpp
	src/tasks/attitudethrustcontroller.cpp
	src/tasks/monitor.cpp
	src/behavior_controllers/bcontrollerexample.cpp
	src/behavior_controllers/takeofflandtest.cpp
	src/behavior_controllers/takeoffgotolandtest.cpp
	src/behavior_controllers/attitudethrusthover.cpp
	src/behavior_controllers/joystick.cpp
	)
target_link_libraries(uri_uav ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
# message(WARNING ${EIGEN_LIBRARIES})


