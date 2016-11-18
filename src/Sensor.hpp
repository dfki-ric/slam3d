#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "Types.hpp"
#include "Logger.hpp"

namespace slam3d
{	
	/**
	 * @class BadMeasurementType
	 * @brief Exception thrown when measurement types do not match.
	 * @details This can happen when a sensor is given a measurement that
	 * originates from a different sensor. In general the sensor is supposed
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
	 * @brief Exception thrown when two measurements could not be matched.
	 */
	class NoMatch: public std::exception
	{
	public:
		NoMatch(const std::string& msg):message(msg){}
		virtual ~NoMatch() throw() {}
		virtual const char* what() const throw()
		{
			return message.c_str();
		}
		
		std::string message;
	};
	
	/**
	 * @class Sensor
	 * @brief Base class for a sensor used in the mapping process.
	 * @details The sensor is responsible for calculating relative poses between
	 * its measurements and for creating representations (maps) from all
	 * readings using the corrected poses from SLAM.
	 */
	class Sensor
	{
	public:
		Sensor(const std::string& n, Logger* l, const Transform& p)
		 :mName(n), mLogger(l), mSensorPose(p){}
		virtual ~Sensor(){}
		
		/**
		 * @brief Get the sensor's name. The name is used to identify
		 * measurements that have been recorded by this sensor.
		 * @return name of the sensor
		 */
		std::string getName() const { return mName; }
		
		/**
		 * @brief Get the sensor's pose in robot coordinate frame.
		 * @return pose sensor pose in robot coordinates
		 */
		Transform getSensorPose() const { return mSensorPose; }
		
		/**
		 * @brief Calculate the estimated transform between two measurements of this sensor.
		 * @param source
		 * @param target
		 * @param odometry estimation of robot movement
		 * @param coarse whether to do a coarse estimate
		 * @throw BadMeasurementType
		 */
		virtual TransformWithCovariance calculateTransform(Measurement::Ptr source,
		                                                   Measurement::Ptr target,
		                                                   Transform odometry,
		                                                   bool coarse = false) const = 0;		
		/**
		 * @brief Creates a virtual measurement at the given pose from a set of vertices.
		 * @param vertices list of vertices that should contain measurements from this sensor
		 * @param pose origin of the virtual measurement
		 * @throw BadMeasurementType
		 */
		virtual Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const = 0;
		
	protected:
		std::string mName;
		Logger* mLogger;
		Transform mSensorPose;
	};
	
	typedef std::map<std::string, Sensor*> SensorList;
}

#endif
