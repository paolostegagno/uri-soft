#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"

#include <tinyxml.h>

#ifndef __URI_UTILITIES_HPP__
#define __URI_UTILITIES_HPP__

namespace uri{
	
	bool getStringParam(ros::NodeHandle &nh, std::string &s, std::string param_name, std::string robot_name);
	
	bool getBoolParam(ros::NodeHandle &nh, bool &s, std::string param_name, std::string robot_name);
	
	bool getIntParam(ros::NodeHandle &nh, int &s, std::string param_name, std::string robot_name);
	
	bool getDoubleParam(ros::NodeHandle &nh, double &s, std::string param_name, std::string robot_name);
	
	/// @brief base class for all options in uri-soft
	class Option{
		public:
			
			/// @brief name of the Option
			std::string name;
			
			/// @brief Constructor
			/// @param[in] nm name of the Option
			Option(std::string nm){
				name=nm;
			}
			
			/// @brief A derived class must provide a function to update its value.
			/// This method is pure virtual and must be implemented in any derived class.
			virtual bool updateThisOption(TiXmlAttribute* att)=0;
			
			/// @brief An option may return a double value.
			/// This method virtual and may be implemented in a child class.
			/// If a child class does not implement this method, then the default returned value is 0.0.
			virtual double getDoubleValue(){
				return 0.0;
			};
			
			/// @brief An option may return a std:string value.
			/// This method virtual and may be implemented in a child class.
			/// If a child class does not implement this method, then the default returned value is "".
			virtual std::string getStringValue(){
				return "";
			};
			
			/// @brief An option may return an int value.
			/// This method virtual and may be implemented in a child class.
			/// If a child class does not implement this method, then the default returned value is 1.
			virtual int getIntValue(){
				return 1;
			};
			
			/// @brief An option may return a bools value.
			/// This method virtual and may be implemented in a child class.
			/// If a child class does not implement this method, then the default returned value is false.
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
	
	

	
	
	/// @brief std::string implementation of the Option
	class OptionString:public Option{
		public:
			
			/// @brief stored value.
			std::string value;
			
			/// @brief complete constructor.
			OptionString (std::string nm, std::string val):Option(nm){
				value = val;
			}
			
			/// @brief update the value of this option based on the cotent of the TiXmlAttribute *att.
			/// @details if the name of *att is equal to the name of the option, then update the value, otherwise do nothing.
			/// @return \b true if the name of the attribute is equal to the name of the option and the value is updated, \b false otherwise
			virtual bool updateThisOption(TiXmlAttribute* att){
				if(att->NameTStr().compare(name)==0){
					value = att->ValueStr();
					return true;
				}
				return false;
			}
			
			/// @brief get the value of the Option as a std::string
			virtual std::string getStringValue(){
				return value;
			};
	};
	
	
	/// @brief int implementation of the Option
	class OptionInt:public Option{
		public:
			
			/// @brief stored value.
			int value;
		
			/// @brief complete constructor.
			OptionInt(std::string nm, int val):Option(nm){
				value = val;
			}
			
			/// @brief update the value of this option based on the cotent of the TiXmlAttribute *att.
			/// @details if the name of *att is equal to the name of the option, then update the value, otherwise do nothing.
			/// @return \b true if the name of the attribute is equal to the name of the option and the value is updated, \b false otherwise
			virtual bool updateThisOption(TiXmlAttribute* att){
				if(att->NameTStr().compare(name)==0){
					if (att->QueryIntValue(&value)==TIXML_SUCCESS) ROS_INFO("  Set %s to %d", att->Name(), value);
					else ROS_ERROR("  %s not a valid int number: %s", att->Name(), att->Value());
					return true;
				}
				return false;
			}
			
			/// @brief get the value of the Option as an int
			virtual int getIntValue(){
				return value;
			};
	};
	
	
	/// @brief double implementation of the Option
	class OptionDouble:public Option{
		public:
			
			/// @brief stored value.
			double value;
		
			/// @brief complete constructor.
			OptionDouble(std::string nm, double val):Option(nm){
				value = val;
			}
			
			/// @brief update the value of this option based on the cotent of the TiXmlAttribute *att.
			/// @details if the name of *att is equal to the name of the option, then update the value, otherwise do nothing.
			/// @return \b true if the name of the attribute is equal to the name of the option and the value is updated, \b false otherwise
			virtual bool updateThisOption(TiXmlAttribute* att){
				if(att->NameTStr().compare(name)==0){
					if (att->QueryDoubleValue(&value)==TIXML_SUCCESS) ROS_INFO("  Set %s to %f", att->Name(), value);
					else ROS_ERROR("  %s not a valid float number: %s", att->Name(), att->Value());
					return true;
				}
				return false;
			}
			
			/// @brief get the value of the Option as a double
			virtual double getDoubleValue(){
				return value;
			};
	};
	
	
	/// @brief bool implementation of the Option
	class OptionBool:public Option{
		public:
			
			/// @brief stored value.
			bool value;
		
			/// @brief complete constructor.
			OptionBool(std::string nm, bool val):Option(nm){
				value = val;
			}
			
			/// @brief update the value of this option based on the cotent of the TiXmlAttribute *att.
			/// @details if the name of *att is equal to the name of the option, then update the value, otherwise do nothing.
			/// @return \b true if the name of the attribute is equal to the name of the option and the value is updated, \b false otherwise
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
			
			/// @brief get the value of the Option as a bool
			virtual bool getBoolValue(){
				return value;
			};
		
	};

	
	
	/// @brief a map associating a string to an Option*
	class OptionVector :public std::map<std::string, Option*>{
		
		public: 
			
			/// @brief add an option of type double to the map
			void addDoubleOption(std::string nm, double val){
				OptionDouble* newopt = new OptionDouble(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}
			
			/// @brief add an option of type std::string to the map
			void addStringOption(std::string nm, std::string val){
				OptionString* newopt = new OptionString(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}

			/// @brief add an option of type int to the map
			void addIntOption(std::string nm, int val){
				OptionInt* newopt = new OptionInt(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}
			
			/// @brief add an option of type bool to the map
			void addBoolOption(std::string nm, bool val){
				OptionBool* newopt = new OptionBool(nm, val);
				this->insert(std::pair<std::string,Option*>(nm,newopt));
			}
			
			/// @brief update an option whose name is equal to the name in att with the value in att
			/// @return \b true if the OptionVector contains an Option with name att and its value is updated, \b false otherwise.
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