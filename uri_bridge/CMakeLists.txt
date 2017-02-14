cmake_minimum_required(VERSION 2.8.3)
project(uri_bridge)

set(CMAKE_CXX_FLAGS "-g -std=c++11")
# set(CMAKE_C '-std=c++11')
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  uri_core
  uri_base
  uri_uav
  uri_sensors
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
		uri_bridge
	CATKIN_DEPENDS
		roscpp
		rospy
		uri_core
		uri_sensors
		uri_uav
	DEPENDS
#		TinyXML
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
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(uri_core
#   src/${PROJECT_NAME}/uri_core.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(uri_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_library(uri_bridge
# src/tasks/dummy_task.cpp
# src/tasks/example_task.cpp
src/bridge_map.cpp
src/tasks/collect3dscan.cpp
src/behavior_controllers/example_behavior_controller.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(uri_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(uri_bridge
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



## Declare a C++ executable
add_executable(test_grid_map
src/test/test_grid_map.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_grid_map uri_bridge)

## Specify libraries to link a library or executable target against
target_link_libraries(test_grid_map
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${OpenCV_LIBRARIES}
  uri_bridge
)


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
