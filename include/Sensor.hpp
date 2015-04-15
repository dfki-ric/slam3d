#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "Measurement.hpp"
#include "Logger.hpp"

#include <vector>
#include <string>

namespace slam
{
	typedef std::vector<Measurement*> MeasurementList;
	
	class BadMeasurementType: public std::exception
	{
	public:
		BadMeasurementType(){}
		virtual const char* what() const throw()
		{
			return "Measurement type does not match sensor type!";
		}
	};
	
	// Forward declaration of GraphMapper
	class GraphMapper;
	
	class Sensor
	{
	public:
		Sensor(std::string n, GraphMapper* m, Logger* l)
		 :mName(n), mMapper(m), mLogger(l){}
		~Sensor(){}
		
		std::string getName(){ return mName; }
	
//	protected:
		void addReading(Measurement* m);
		
		MeasurementList getAllReadings();
		Measurement* getLastReading();
		
		virtual TransformWithCovariance calculateTransform(Measurement* source, Measurement* target) const = 0;
		
	protected:
		std::string mName;
		GraphMapper* mMapper;
		MeasurementList mReadings;
		Logger* mLogger;
	};
}

#endif