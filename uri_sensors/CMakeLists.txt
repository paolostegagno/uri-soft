cmake_minimum_required(VERSION 2.8.3)
project(uri_sensors)

set(CMAKE_CXX_FLAGS "-g -std=c++11")
# set(CMAKE_C '-std=c++11')
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  uri_core
  uri_base
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)


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
		uri_sensors
	CATKIN_DEPENDS roscpp rospy uri_core uri_base
	DEPENDS
		TinyXML
		Eigen
		opencv
)



###########
## Build ##
###########

# message(${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(uri_sensors
src/resources/laserscanner.cpp
src/resources/laserscannergazebo.cpp
src/resources/videostream.cpp
)

## Add cmake target dependencies of the library
add_dependencies(uri_sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(uri_sensors
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN_LIBRARIES}
)

## Declare a C++ executable
# add_executable(uri_sensors_executable
#   src/${PROJECT_NAME}/main.cpp
# )

## Add cmake target dependencies of the executable
# add_dependencies(uri_sensors_executable ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(uri_sensors_executable
#   ${catkin_LIBRARIES}
#   ${TinyXML_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS uri_core uri_core_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uri_core.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
