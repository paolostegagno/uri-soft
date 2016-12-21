#!/usr/bin/env python
import rospy
import rospkg
import sys
import os.path

from shutil import copyfile

import fileinput



if __name__=="__main__":
	if len(sys.argv) < 2:
		print("USAGE ERROR!: not enough arguments")
		print("USAGE       : my_node.py new_task_name destitation_package")
	else:
		taskname = sys.argv[1]
		packagename = sys.argv[2]
		print 'I\'ll create a task named',taskname,'in',  packagename

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
source_path = rospack.get_path('uri_example')
source_path_inc = source_path + '/include/uri_example/tasks/example_task.hpp'
source_path_src = source_path + '/src/tasks/example_task.cpp'
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
destination_path_inc = destination_path + '/include/' + packagename + '/tasks'
if not os.path.exists(destination_path_inc):
	os.makedirs(destination_path_inc)
	print "- created folder " + destination_path_inc
else:
	print "- found folder " + destination_path_inc
	
destination_path_src = destination_path + '/src/tasks'
if not os.path.exists(destination_path_src):
	os.makedirs(destination_path_src)
	print "- created folder " + destination_path_src
else:
	print "- found folder " + destination_path_src

destination_path_hpp = destination_path_inc+"/"+taskname.lower()+".hpp"
destination_path_cpp = destination_path_src+"/"+taskname.lower()+".cpp"

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
filedata = filedata.replace("ExampleTask", taskname) # fix class name
filedata = filedata.replace("EXAMPLE_TASK", taskname.upper()) # fix the initial declaration ifndef define
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
filedata = filedata.replace("ExampleTask", taskname) # fix class name
filedata = filedata.replace("example_task", taskname.lower()) # fix class name
filedata = filedata.replace("EXAMPLE_TASK", taskname.upper()) # fix the initial declaration ifndef define
# Write the file out again
with open(destination_path_cpp, 'w') as file:
	file.write(filedata)


