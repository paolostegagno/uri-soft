cmake_minimum_required(VERSION 2.8.3)
project(uri_gazebo)

set(CMAKE_CXX_FLAGS "-g -std=c++11")
add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  rosbag
  gazebo_msgs
  tf_conversions
  uri_base
  uri_core
)

find_package(mav_msgs QUIET)


catkin_package(  
  CATKIN_DEPENDS 
    geometry_msgs 
    mav_msgs
    roscpp 
    sensor_msgs 
    rosbag
    gazebo_msgs
    tf_conversions
    uri_core
    uri_base
#     telekyb_msgs
#     telekyb_srvs
  DEPENDS Eigen
)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
src/tasks/directposecommandgz.cpp
src/behavior_controllers/commandposebc.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})



if(mav_msgs_FOUND)
  add_executable(sitl_gazebo_interface src/sitl_gazebo_interface.cpp)
  target_link_libraries(sitl_gazebo_interface ${catkin_LIBRARIES})
else(mav_msgs_FOUND)
  message(WARNING "Missing package 'mav_msgs', this package will not be built!")
endif(mav_msgs_FOUND)


