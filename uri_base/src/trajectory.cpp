
#include "uri_base/trajectory.hpp"



namespace uri_base{


	
	void TwoByNMatrix::update(sensor_msgs::LaserScan l){
// 		std::cout << " " << l.intensities.size() << " " << l.ranges.size() << std::endl;
		
		for (int j=0; j < l.intensities.size(); j++ ){
			
			double dist = l.ranges[j];
			double intensity = l.intensities[j];
			int cell_index = ceil(dist/_step);
			if(cell_index >= 0 and cell_index < _cell_number){
// 						std::cout << j << " " << cell_index << std::endl;
				
				_measurements_counter[cell_index]++;
				_mean_intensities[cell_index] = (_measurements_counter[cell_index]-1)*_mean_intensities[cell_index]/_measurements_counter[cell_index]
																				+ intensity/_measurements_counter[cell_index];
			}
		}
// 				std::cout << std::endl;
	}

	double TwoByNMatrix::distance_from_mean_intensity(double dist, double intensity){
		int index = dist / _step;
// 		std::cout << "c1  " << dist << " " << intensity << " " << index << " " << _min << " " << _max << " " << _cell_number << std::endl;
		if (index >= 0 && index < _cell_number) return intensity - _mean_intensities[index];
		else return 0;
	}

	
	void TwoByNMatrix::print(std::stringstream &ss){
		
		ss << _step << " " << _max << " " << _min << " " << _cell_number;
		for (int j=0; j < _cell_number; j++ ){
			ss << " " << _mean_intensities[j];
		}
		for (int j=0; j < _cell_number; j++ ){
			ss << " " << _measurements_counter[j];
		}
		ss << std::endl;
	}

}; // end namespace




