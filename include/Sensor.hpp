#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "Measurement.hpp"
#include "Logger.hpp"

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
		Sensor(std::string n, GraphMapper* m, Logger* l)
		 :mName(n), mMapper(m), mLogger(l){}
		~Sensor(){}
	
	protected:
		void addReading(Measurement* m);
		
		MeasurementList getAllReadings();
		Measurement* getLastReading();
		
	protected:
		std::string mName;
		GraphMapper* mMapper;
		MeasurementList mReadings;
		Logger* mLogger;
	};
}

#endif