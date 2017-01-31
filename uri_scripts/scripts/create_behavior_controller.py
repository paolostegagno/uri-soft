#!/usr/bin/env python
import rospy
import rospkg
import sys
import os.path

from shutil import copyfile

import fileinput



if __name__=="__main__":
	if len(sys.argv) < 3:
		print("USAGE ERROR!: not enough arguments")
		print("USAGE       : rosrun uri_scripts create_task.py new_behaviorcontroller_name destitation_package")
		sys.exit(1)
	else:
		behaviorcontrollername = sys.argv[1]
		packagename = sys.argv[2]
		print 'I\'ll create a behavior controller named',behaviorcontrollername,'in',  packagename

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
source_path = rospack.get_path('uri_example')
source_path_inc = source_path + '/include/uri_example/behavior_controllers/example_behavior_controller.hpp'
source_path_src = source_path + '/src/behavior_controllers/example_behavior_controller.cpp'
print "- found " + source_path

if os.path.isfile(source_path_inc):
	print "- found header file", source_path_inc
else:
	error_string = "FATAL ERROR: could not locate " + source_path_inc
	sys.exit(error_string)

if os.path.isfile(source_path_src):
	print "- found source file", source_path_src
else:
	error_string = "FATAL ERROR: could not locate " + source_path_src
	sys.exit(error_string)


destination_path = rospack.get_path(packagename)
destination_path_inc = destination_path + '/include/' + packagename + '/behavior_controllers'
if not os.path.exists(destination_path_inc):
	os.makedirs(destination_path_inc)
	print "- created folder " + destination_path_inc
else:
	print "- found folder " + destination_path_inc
	
destination_path_src = destination_path + '/src/behavior_controllers'
if not os.path.exists(destination_path_src):
	os.makedirs(destination_path_src)
	print "- created folder " + destination_path_src
else:
	print "- found folder " + destination_path_src

destination_path_hpp = destination_path_inc+"/"+behaviorcontrollername.lower()+".hpp"
destination_path_cpp = destination_path_src+"/"+behaviorcontrollername.lower()+".cpp"

copyfile(source_path_inc, destination_path_hpp)
copyfile(source_path_src, destination_path_cpp)

print "--- Created "+destination_path_hpp
print "--- Created "+destination_path_cpp


# FIX THE hpp file
# Read in the file
filedata = None
with open(destination_path_hpp, 'r') as file :
	filedata = file.read()
# Replace the target string
filedata = filedata.replace("ExampleBehaviorController", behaviorcontrollername) # fix class name
filedata = filedata.replace("EXAMPLE_BEHAVIOR_CONTROLLER", behaviorcontrollername.upper()) # fix the initial declaration ifndef define
filedata = filedata.replace("uri_example", packagename) # fix the namespace
# Write the file out again
with open(destination_path_hpp, 'w') as file:
	file.write(filedata)


# FIX THE cpp file
# Read in the file
filedata = None
with open(destination_path_cpp, 'r') as file :
	filedata = file.read()
# Replace the target string
filedata = filedata.replace("uri_example", packagename) # fix the namespace
filedata = filedata.replace("ExampleBehaviorController", behaviorcontrollername) # fix class name
filedata = filedata.replace("example_behavior_controller", behaviorcontrollername.lower()) # fix class name
filedata = filedata.replace("EXAMPLE_BEHAVIOR_CONTROLLER", behaviorcontrollername.upper()) # fix the initial declaration ifndef define
# Write the file out again
with open(destination_path_cpp, 'w') as file:
	file.write(filedata)


