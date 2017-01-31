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
		print("USAGE       : rosrun uri_scripts create_task.py new_resource_name destitation_package")
		sys.exit(1)
	else:
		resourcename = sys.argv[1]
		packagename = sys.argv[2]
		print 'I\'ll create a Resource named',resourcename,'in',  packagename

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
source_path = rospack.get_path('uri_example')
source_path_inc = source_path + '/include/uri_example/resources/example_resource.hpp'
source_path_src = source_path + '/src/resources/example_resource.cpp'
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
print packagename
destination_path_inc = destination_path + '/include/' + packagename + '/resources'
if not os.path.exists(destination_path_inc):
	os.makedirs(destination_path_inc)
	print "- created folder " + destination_path_inc
else:
	print "- found folder " + destination_path_inc
	
destination_path_src = destination_path + '/src/resources'
if not os.path.exists(destination_path_src):
	os.makedirs(destination_path_src)
	print "- created folder " + destination_path_src
else:
	print "- found folder " + destination_path_src

destination_path_hpp = destination_path_inc+"/"+resourcename.lower()+".hpp"
destination_path_cpp = destination_path_src+"/"+resourcename.lower()+".cpp"

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
filedata = filedata.replace("ExampleResource", resourcename) # fix class name
filedata = filedata.replace("EXAMPLERESOURCE", resourcename.upper()) # fix the initial declaration ifndef define
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
filedata = filedata.replace("ExampleResource", resourcename) # fix class name
filedata = filedata.replace("example_resource", resourcename.lower()) # fix class name
filedata = filedata.replace("EXAMPLERESOURCE", resourcename.upper()) # fix the initial declaration ifndef define
# Write the file out again
with open(destination_path_cpp, 'w') as file:
	file.write(filedata)


