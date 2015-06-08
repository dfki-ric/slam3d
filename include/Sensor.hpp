#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "Types.hpp"
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
	
	/**
	 * @class NoMatch
	 * @author Sebastian Kasperski
	 * @date 04/21/15
	 * @file Sensor.hpp
	 * @brief 
	 */
	class NoMatch: public std::exception
	{
	public:
		NoMatch(){}
		virtual const char* what() const throw()
		{
			return "Could not match measurements from sensor!";
		}
	};
	
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
		Sensor(std::string n, Logger* l)
		 :mName(n), mLogger(l){}
		virtual ~Sensor(){}
		
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
		 * @param guess Initial estiamte of transform between source and target
		 */
		virtual TransformWithCovariance calculateTransform(Measurement* source, Measurement* target, Transform guess) const = 0;
		
	protected:
		std::string mName;
		Logger* mLogger;
	};
}

#endif