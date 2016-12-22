

#include <string>
#include <thread>



#include <pluginlib/class_list_macros.h>
#include <uri_core/resource.hpp>

#include <Eigen/Geometry>

#include <unistd.h>   //_getch
#include <termios.h>  //_getch


#ifndef __KEYBOARD_HPP__
#define __KEYBOARD_HPP__

#define KEYBOARD_BUFFER_LENGTH 100

using namespace uri;

namespace uri_base {
	
	
	char getch(){
    /*#include <unistd.h>   //_getch*/
    /*#include <termios.h>  //_getch*/
    char buf=0;
    struct termios old={0};
    fflush(stdout);
    if(tcgetattr(0, &old)<0)
        perror("tcsetattr()");
    old.c_lflag&=~ICANON;
    old.c_lflag&=~ECHO;
    old.c_cc[VMIN]=1;
    old.c_cc[VTIME]=0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattr ICANON");
    if(read(0,&buf,1)<0)
        perror("read()");
    old.c_lflag|=ICANON;
    old.c_lflag|=ECHO;
    if(tcsetattr(0, TCSADRAIN, &old)<0)
        perror ("tcsetattr ~ICANON");
//     printf("%c\n",buf);
    return buf;
 }
	
	
/// @brief A module to read the keyboard.
/// @details This Resource will read the keys of the keyboard provide them to a module that reads it.
class Keyboard: public Resource{
		
		/// @brief Madatory initialization method.
		/// @details this method is called once after the instantiation is created.
		void _init(){
			_active = true;
			getcharacters = new std::thread(&Keyboard::read_cycle, this);
		}
		
		/// @brief This is a circular buffer.
		/// @details Its lenght is specified by an option.
		char _buffer[KEYBOARD_BUFFER_LENGTH];
		
		/// @brief Current buffer start.
		/// @details This points to the first character not read by an external module.
		int _buffer_start;
		
		/// @brief Current buffer end.
		/// @details This points to the last character not read by an external module.
		int _buffer_end;
		
		/// @brief This @b bool acts as a mutex for the data.
		bool _busy;
		
		/// @brief When this bool is true, the resource is active.
		/// @details If _active is set at false, the thread will stop.
		bool _active;
		
		
		
		std::thread* getcharacters;
		
		void read_cycle() {
			while (this->_active){
				usleep(100);
				while (_busy){
					usleep(100);
				}
				_buffer[_buffer_end] = getch();
				_busy = true;
			std::cout << "reading " << _buffer_end << " " << _buffer_start << std::endl; 
				_buffer_end++;
				if (_buffer_end==KEYBOARD_BUFFER_LENGTH){
					_buffer_end=0;
				}
				if (_buffer_start == _buffer_end){
					_buffer_start++;
				}
			std::cout << "reading end " << _buffer_end << " " << _buffer_start << std::endl; 
				_busy = false;
			}
		}
		
		
	public:
		
		/// @brief Do-nothing constructor
		/// @details This method does not build anything. To be followed by initialize(ros::NodeHandle &n).
		Keyboard(){
			_name = "uri_base::Keyboard";
			_busy = false;
			_buffer_end = 0;
			_buffer_start = 0;
			_active = false;
		};
		
		
		/// @brief Activate reading from the keyboard.
		void activate(){
			this->_init();
		}
		
		/// @brief Deactivate reading from the keyboard.
		void deactivate(){
			_active = false;
		}
		
		/// @brief Gets the data stored in the buffer.
		/// @param[out] out If return is > 0, the data stored in the shared memory are copied here.
		/// @param[in] timeout_s Maximum time limit to get the data.
		/// @return \b true if the data stored in the shared memory are available to be copied within the time limit, \b false otherwise.
		int get(char* &out){
			int num, a = 0;
			if (_busy){
				usleep(1000);
			}
			if (_busy){
				return 0;
			}
			_busy = true;
		std::cout << "getting "  << _buffer_end << " " << _buffer_start  << std::endl; 
			if (_buffer_end > _buffer_start){
				num = _buffer_end - _buffer_start;
				out = new char[num];
				for (int i = _buffer_start; i<_buffer_end; i++){
					out[a] = _buffer[i];
					a++;
				}
				_buffer_start = _buffer_end;
			} 
			else if (_buffer_end == _buffer_start){
				num = 0;
			}
			else if (_buffer_end < _buffer_start){
				num = KEYBOARD_BUFFER_LENGTH - _buffer_start + _buffer_end;
				out = new char[num];
				for (int i = _buffer_start; i<KEYBOARD_BUFFER_LENGTH; i++){
					out[a] = _buffer[i];
					a++;
				}
				for (int i = 0; i<_buffer_end; i++){
					out[a] = _buffer[i];
					a++;
				}
				_buffer_start = _buffer_end;
			}
		std::cout << "getting end "  << _buffer_end << " " << _buffer_start  << std::endl; 
			_busy = false;
			return num;
			
		}
		
		
		
};


}

  PLUGINLIB_EXPORT_CLASS(uri_base::Keyboard, uri::Resource)




#endif






