
#include <uri_core/resource.hpp>

namespace uri{
	
	
	std::string& Resource::type(){
		return _type;
	}
	
	Resource::Resource(){
		
	}
	
	Resource::Resource(ros::NodeHandle &_n):Element(_n){
		
	}
	
	Resource* Resource::ptr(){
		return this;
	}
	
	Resource* ResourceVector::get_resource_ptr(std::string nm){
		
		for (int i=0; i < this->size();i++){
			
// 			std::cout << (*this)[i]->name() << std::endl;
			if ( (*this)[i]->name().compare(nm)==0 ){
				ROS_INFO("  Found mandatory resource %s.", nm.c_str());
				return (*this)[i]->ptr();
			}
		}
		ROS_ERROR("  Cannot find mandatory resource %s.", nm.c_str());
		return NULL;
	}

	
	
	Resource* ResourceVector::get_resource_ptr(std::string nm, std::string nam){
		
		for (int i=0; i < this->size();i++){
			
// 			std::cout << (*this)[i]->name() << std::endl;
			if ( (*this)[i]->name().compare(nm)==0 ){
				std::string type;
				bool type_option_exists = (*this)[i]->option("type", type);
				
				if (type_option_exists) {
					if ( type.compare(nam) == 0 ){
						ROS_INFO("  Found mandatory resource %s %s.", nm.c_str(), nam.c_str());
						return (*this)[i]->ptr();
					}
				}
			}
		}
		ROS_ERROR("  Cannot find mandatory resource %s of type %s.", nm.c_str(), nam.c_str());
		return NULL;
	}


	
};


