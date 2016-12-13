#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/element.hpp>

namespace uri{

  

Element::Element(){
	
}


Element::Element(ros::NodeHandle &_n){
	n=&_n;
}


std::string& Element::name(){
	return _name;
}

void Element::init(ros::NodeHandle &nh, TiXmlAttribute* attribute){
	

	// first read all the options and stores them in _options
	int i=0;
	if ( !attribute) {
	}
	else {
		int ival;
		double dval;
		while (attribute)
		{
			_options.updateOption(attribute);
			i++;
			attribute=attribute->Next();
		}
	}
	
	// then take the nodehandle
	n = new ros::NodeHandle(nh);
	
	
	
	_init();
}




};


