#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <tinyxml.h>

#ifndef __URI_UTILITIES_HPP__
#define __URI_UTILITIES_HPP__

namespace uri{
	
	void ERROR();
	
	void WARNING(std::string msg);
	
	bool getStringParam(ros::NodeHandle &nh, std::string &s, std::string param_name, std::string robot_name);
	
	bool getBoolParam(ros::NodeHandle &nh, bool &s, std::string param_name, std::string robot_name);
	
	bool getIntParam(ros::NodeHandle &nh, int &s, std::string param_name, std::string robot_name);
	
	bool getDoubleParam(ros::NodeHandle &nh, double &s, std::string param_name, std::string robot_name);
	
	
	class Option{
		public:
			
			std::string name;
			
			Option(std::string nm){
				name=nm;
			}
			
			virtual bool updateThisOption(TiXmlAttribute* att)=0;
			
			virtual double getDoubleValue(){
				return 0.0;
			};

			virtual std::string getStringValue(){
				return "";
			};

			virtual int getIntValue(){
				return 1;
			};

			virtual bool getBoolValue(){
				return false;
			};

	};
	
	
	
/*	
	template <class type> class OptionGen: public Option{
		public:
			
			type value;
			
			OptionGen(std::string nm, type val):Option(nm){
				value=val;
			}
			
			virtual void updateThisOption(std::string nm, type val){
				if(nm.compare(name)==0){
					value = val;
				}
			}
	};*/
	
	

	
	
	
	class OptionString:public Option{
		public:
			
			std::string value;
			
			OptionString (std::string nm, std::string val):Option(nm){
				value = val;
			}
			
			virtual bool updateThisOption(TiXmlAttribute* att){
				if(att->NameTStr().compare(name)==0){
					value = att->ValueStr();
					return true;
				}
				return false;
			}
			
			virtual std::string getStringValue(){
				return value;
			};
	};
	
	class OptionInt:public Option{
		public:
			
			int value;
		
			OptionInt(std::string nm, int val):Option(nm){
				value = val;
			}
			
			virtual bool updateThisOption(TiXmlAttribute* att){
				if(att->NameTStr().compare(name)==0){
					if (att->QueryIntValue(&value)==TIXML_SUCCESS) ROS_INFO("  Set %s to %d", att->Name(), value);
					else ROS_ERROR("  %s not a valid int number: %s", att->Name(), att->Value());
					return true;
				}
				return false;
			}
			
			
			virtual int getIntValue(){
				return value;
			};
	};
	
	class OptionDouble:public Option{
		public:
			
			double value;
		
			OptionDouble(std::string nm, double val):Option(nm){
				value = val;
			}
			
			virtual bool updateThisOption(TiXmlAttribute* att){
				if(att->NameTStr().compare(name)==0){
					if (att->QueryDoubleValue(&value)==TIXML_SUCCESS) ROS_INFO("  Set %s to %f", att->Name(), value);
					else ROS_ERROR("  %s not a valid float number: %s", att->Name(), att->Value());
					return true;
				}
				return false;
			}
			
			virtual double getDoubleValue(){
				return value;
			};
	};
	
	
	class OptionBool:public Option{
		public:
			
			bool value;
		
			OptionBool(std::string nm, bool val):Option(nm){
				value = val;
			}
			
			virtual bool updateThisOption(TiXmlAttribute* att){
				if(att->NameTStr().compare(name)==0){
					if (att->ValueStr().compare("true")==0) {
						value = true;
						ROS_INFO("  Set %s to true", att->Name());
					}
					else if (att->ValueStr().compare("false")==0) {
						value = false;
						ROS_INFO("  Set %s to false", att->Name());
					}
					else {
						ROS_ERROR("  %s not a valid bool value: %s", att->Name(), att->ValueStr().c_str());
					}
					return true;
				}
				return false;
			}
			
			
			virtual bool getBoolValue(){
				return value;
			};
		
	};

	
	
	
		class OptionVector :public std::map<std::string, Option*>{
		
		public: 
			
			void addDoubleOption(std::string nm, double val){
				OptionDouble* newopt = new OptionDouble(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}
			
			void addStringOption(std::string nm, std::string val){
				OptionString* newopt = new OptionString(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}

			void addIntOption(std::string nm, int val){
				OptionInt* newopt = new OptionInt(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}
			
			void addBoolOption(std::string nm, bool val){
				OptionBool* newopt = new OptionBool(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}
			
			bool updateOption(TiXmlAttribute* att){
				
				std::map<std::string,Option*>::iterator it;
				it = this->find(att->NameTStr());
				if (it != this->end()){
					it->second->updateThisOption(att);
					return true;
				}
				ROS_ERROR("  Option %s=%s set in the configuration file but not declared in the option vector.", att->Name(), att->Value());
				return false;
			}
			
			

		
	};

		
	
	
	
	
}

#endif