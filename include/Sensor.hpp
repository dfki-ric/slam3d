#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "Measurement.hpp"
#include "Logger.hpp"

#include <vector>
#include <string>

namespace slam
{	
	/**
	 * @class BadMeasurementType
	 * @author Sebastian Kasperski
	 * @date 04/16/15
	 * @file Sensor.hpp
	 * @brief Exception thrown when the sensor is provided with a measurement
	 * that originates from a different sensor. In general the sensor is supposed
	 * to operate on its own measurements only. However special implementations
	 * could allow to register their readings with those from selected other
	 * sensors. This case should be documented in the specialization class.
	 */
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
	
	/**
	 * @class Sensor
	 * @author Sebastian Kasperski
	 * @date 04/16/15
	 * @file Sensor.hpp
	 * @brief Abstact base class for a Sensor used in the mapping process.
	 * The sensor is responsible for calculating relative poses between its
	 * measurements and for creating representations (maps) from all
	 * readings using the corrected poses from SLAM.
	 */
	class Sensor
	{
	public:
		Sensor(std::string n, GraphMapper* m, Logger* l)
		 :mName(n), mMapper(m), mLogger(l){}
		~Sensor(){}
		
		/**
		 * @brief Get the sensor's name. The name is used to identify
		 * measurements that have been recorded by this sensor.
		 * @return name
		 */
		std::string getName(){ return mName; }
		
		/**
		 * @brief Calculate the estimated transform between two measurements
		 * of this sensor.
		 * @throw BadMeasurementType
		 * @param source
		 * @param target
		 */
		virtual TransformWithCovariance calculateTransform(Measurement* source, Measurement* target) const = 0;
		
	protected:
		std::string mName;
		GraphMapper* mMapper;
		Logger* mLogger;
	};
}

#endif