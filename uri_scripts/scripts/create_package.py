#!/usr/bin/env python
import rospy
import rospkg
import sys
import os.path

from shutil import copyfile

import fileinput



if __name__=="__main__":
	if len(sys.argv) < 2:
		sys.exit("USAGE ERROR!: not enough arguments\nUSAGE       : my_node.py new_package_name")
	else:
		packagename = sys.argv[1]
		print 'I\'ll create a package named',  packagename

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for rospy_tutorials
uri_example_path = rospack.get_path('uri_example')
#source_path_inc = source_path + '/include/uri_example/tasks/example_task.hpp'
#source_path_src = source_path + '/src/tasks/example_task.cpp'
if uri_example_path.endswith('uri_example'):
	uri_soft_path = uri_example_path[:-11]
destination_path = uri_soft_path + packagename
print "- found uri_example package : " + uri_example_path
print "- found uri_soft path       : " + uri_soft_path
print "- I will create new package : " + destination_path

if not os.path.exists(destination_path):
	os.makedirs(destination_path)
	os.makedirs(destination_path + "/src")
	os.makedirs(destination_path + "/include/" + packagename)
	os.makedirs(destination_path + "/launch")
	os.makedirs(destination_path + "/configfiles")
else:
	sys.exit("ERROR: " + destination_path + " already exists!\nExecution terminated.")

package_cmakelists = uri_soft_path + packagename + "/CMakeLists.txt"
package_package = uri_soft_path + packagename + "/package.xml"
package_plugins = uri_soft_path + packagename + "/" + packagename + "_plugins.xml"

copyfile(uri_example_path + "/example_CMakeLists.txt", package_cmakelists)
copyfile(uri_example_path + "/package.xml", package_package)
copyfile(uri_example_path + "/task_plugins.xml", package_plugins)

# FIX THE CMakeLists
# Read in the file
filedata = None
with open(package_cmakelists, 'r') as file :
	filedata = file.read()
# Replace the target string
filedata = filedata.replace("uri_example", packagename) # fix package name name
# Write the file out again
with open(package_cmakelists, 'w') as file:
	file.write(filedata)
print "- created file : " + package_cmakelists


# FIX THE package file
# Read in the file
filedata = None
with open(package_package, 'r') as file :
	filedata = file.read()
# Replace the target string
filedata = filedata.replace("uri_example", packagename) # fix package name name
filedata = filedata.replace("task_plugins.xml", packagename + "_plugins.xml") # fix package name name
# Write the file out again
with open(package_package, 'w') as file:
	file.write(filedata)
print "- created file : " + package_package


# FIX THE plugins file
# Read in the file
filedata = None
with open(package_plugins, 'r') as file :
	filedata = file.read()
# Replace the target string
filedata = filedata.replace("uri_example", packagename) # fix package name name
# Write the file out again
with open(package_plugins, 'w') as file:
	file.write(filedata)
print "- created file : " + package_plugins


print "Done!"





#if os.path.isfile(source_path_inc):
	#print "- found header file", source_path_inc
#else:
	#error_string = "FATAL ERROR: could not locate " + source_path_inc
	#sys.exit(error_string)

#if os.path.isfile(source_path_src):
	#print "- found source file", source_path_src
#else:
	#error_string = "FATAL ERROR: could not locate " + source_path_src
	#sys.exit(error_string)


#destination_path = rospack.get_path(packagename)
#print packagename
#print packagename
#print packagename
#print packagename
#print packagename
#print packagename
#destination_path_inc = destination_path + '/include/' + packagename + '/tasks'
#if not os.path.exists(destination_path_inc):
	#os.makedirs(destination_path_inc)
	#print "- created folder " + destination_path_inc
#else:
	#print "- found folder " + destination_path_inc
	
#destination_path_src = destination_path + '/src/tasks'
#if not os.path.exists(destination_path_src):
	#os.makedirs(destination_path_src)
	#print "- created folder " + destination_path_src
#else:
	#print "- found folder " + destination_path_src

#destination_path_hpp = destination_path_inc+"/"+taskname.lower()+".hpp"
#destination_path_cpp = destination_path_src+"/"+taskname.lower()+".cpp"

#copyfile(source_path_inc, destination_path_hpp)
#copyfile(source_path_src, destination_path_cpp)

#print "--- Created "+destination_path_hpp
#print "--- Created "+destination_path_cpp


## FIX THE hpp file
## Read in the file
#filedata = None
#with open(destination_path_hpp, 'r') as file :
	#filedata = file.read()
## Replace the target string
#filedata = filedata.replace("ExampleTask", taskname) # fix class name
#filedata = filedata.replace("EXAMPLE_TASK", taskname.upper()) # fix the initial declaration ifndef define
#filedata = filedata.replace("uri_example", packagename) # fix the namespace
## Write the file out again
#with open(destination_path_hpp, 'w') as file:
	#file.write(filedata)


## FIX THE cpp file
## Read in the file
#filedata = None
#with open(destination_path_cpp, 'r') as file :
	#filedata = file.read()
## Replace the target string
#filedata = filedata.replace("uri_example", packagename) # fix the namespace
#filedata = filedata.replace("ExampleTask", taskname) # fix class name
#filedata = filedata.replace("example_task", taskname.lower()) # fix class name
#filedata = filedata.replace("EXAMPLE_TASK", taskname.upper()) # fix the initial declaration ifndef define
## Write the file out again
#with open(destination_path_cpp, 'w') as file:
	#file.write(filedata)


