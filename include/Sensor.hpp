#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "Measurement.hpp"

#include <vector>
#include <string>

namespace slam
{
	typedef std::vector<Measurement*> MeasurementList;
	
	// Forward declaration of GraphMapper
	class GraphMapper;
	
	class Sensor
	{
	public:
		Sensor(std::string n, GraphMapper* m):mName(n),mMapper(m){}
		~Sensor(){}
	
	protected:
		void addReading(Measurement* m);
		
		MeasurementList getAllReadings();
		Measurement* getLastReading();
		
	protected:
		std::string mName;
		GraphMapper* mMapper;
		MeasurementList mReadings;
	};
}

#endif