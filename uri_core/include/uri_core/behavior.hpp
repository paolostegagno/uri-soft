#include <iostream>
#include <vector>
//#include <thread>

#include "ros/ros.h"
#include <uri_core/utilities.hpp>
#include <uri_core/task.hpp>

#ifndef __URI_BEHAVOUR_HPP__
#define __URI_BEHAVOUR_HPP__

namespace uri{
	
	/// @brief A Behavior of the robot as collection of Task.
	/// @details This is the base class for the many behaviors that a robot can assume during operation.
	/// A Behavior is defined as a collection of Task that the robot must execute at constant time intervals.
	class Behavior{
		
		std::string _name;
		
		std::vector<boost::shared_ptr<uri::Task> > _task;
		
		int _counter;
		
	public:
		
		/// @brief Default constructor.
		/// @details It does nothing.
		Behavior();
		
		/// @brief Full contructor.
		/// @details This constructor sets the name of the Behavior, and create a Task list extracting the required Tasks out of a vector of Task pointers.
		/// @param[in] &name The name of the behavior.
		/// @param[in] &tasks A vector of pointers to allocated existing uri::Task's.
		/// @param[in] &required_tasks A string containing a list of all required tasks.
		Behavior(std::string &name, std::vector<boost::shared_ptr<uri::Task> > &tasks, const std::string &required_tasks);
		
		/// @brief Get the i-th task.
		/// @param[in] i The number of the required Task.
		/// @return A boot pointer to the i-th Task.
		boost::shared_ptr<uri::Task> task(int i){
			return _task[i];
		}
		
		/// @brief Get number of tasks in the behavior.
		/// @return An int corresponding to the number of tasks in the behavior.
		int num_tasks(){
			return _task.size();
		}
		
		/// @brief Get the name of the behavior.
		/// @return A std::string corresponding to the name of the behavior.
		std::string& name(){
			return _name;
		}
		
		/// @brief Get the name of the behavior.
		/// @return A std::string corresponding to the name of the behavior.
		int contains_task(std::string &tn);
		
		/// @brief Set an option of type double.
		/// @return Returns \b true if the option exists and has been set, \b false otherwise.
		bool set_option_double(std::string &oname, std::string &tname, double value);
		
		/// @brief Set an option of type double.
		/// @return Returns \b true if the option exists and has been set, \b false otherwise.
		bool set_option_double(const char* oname_c, const char* tname_c, double value);
		
		/// @brief Set an option of type bool.
		/// @return Returns \b true if the option exists and has been set, \b false otherwise.
		bool set_option_bool(std::string &oname, std::string &tname, bool value);
		
		/// @brief Set an option of type string.
		/// @return Returns \b true if the option exists and has been set, \b false otherwise.
		bool set_option_string(std::string &oname, std::string &tname, std::string value);
		
		/// @brief Set an option of type int.
		/// @return Returns \b true if the option exists and has been set, \b false otherwise.
		bool set_option_int(std::string &oname, std::string &tname, int value);
		
		/// @brief Print the behavior.
		/// @details Prints on screen (through cout) interesting data about the behavior, i.e., name, # and names of Tasks. 
		void print();
		
		/// @brief Whether or not the Behavior requires termination.
		/// @return \b true if a Task has required termination, \b false otherwise
		bool terminate();
		
		/// @brief returns the number of times the Behavior has been activated.
		/// @return returns the number of times the Behavior has been activated.
		int counter(){
			return _counter;
		}
		
		/// @brief set the counter of number of times the Behavior has been activated.
		/// @param[in] c the number to be set.
		void set_counter(int c){
			_counter = c;
		}
		
		/// @brief reset the counter of number of times the Behavior has been activated.
		void reset_counter(){
			_counter = 0;
		}
		
		/// @brief increase current activation counter.
		void increase_counter(){
			_counter++;
		}
		
		
	};
	
};

#endif

