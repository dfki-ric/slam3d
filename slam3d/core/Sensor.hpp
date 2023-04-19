// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2017 S. Kasperski
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SLAM_SENSOR_HPP
#define SLAM_SENSOR_HPP

#include "Types.hpp"

#include <set>

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
	 * @details This is not necessarily a problem if it happens once in a while.
	 * But when matching fails regularly, the mapping can fail and matching
	 * parameters should be changed by the user.
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
	
	class Mapper;
	class Logger;
	
	/**
	 * @class Sensor
	 * @brief Abstract base class for a sensor used in the mapping process.
	 * @details The sensor is responsible for adding new measurements to the
	 * graph holding its measurements and for creating representations (maps)
	 * from all readings using the corrected poses from SLAM.
	 */
	class Sensor
	{
	public:
		Sensor(const std::string& n, Logger* l)
		 :mMapper(NULL), mLogger(l), mName(n), mLastVertex(0), mCovarianceScale(1.0){}
		virtual ~Sensor(){}
		
		/**
		 * @brief Set the mapper that this sensor is used by.
		 * @param m Mapper
		 */
		void setMapper(Mapper* m) { mMapper = m; }
		
		/**
		 * @brief Get the sensor's name. The name is used to identify
		 * measurements that have been recorded by this sensor.
		 * @return name of the sensor
		 */
		std::string getName() const { return mName; }
		
		/**
		 * @brief Set minimal change in pose between adjacent nodes.
		 * @param t Minimum translation between nodes (in meter).
		 * @param r Minimum rotation between nodes (in rad).
		 */
		void setMinPoseDistance(float t, float r);
		
		/**
		 * @brief Checks the given transformation using the previously set
		 * translation and rotation thresholds.
		 * @param t Relative motion since the last scan was added.
		 * @return Whether a new scan should be added.
		 */
		bool checkMinDistance(const Transform &t);
		
		/**
		 * @brief Get the ID of the last vertex from this sensor.
		 * @return Vertex ID
		 */
		IdType getLastVertexId(){ return mLastVertex; }

		/**
		 * @brief Set the covariance scale for measurements of this sensor.
		 * @param s
		 */
		void setCovarianceScale(ScalarType s){ mCovarianceScale = s; }

		/**
		 * @brief Use measurements of this sensor for neighbor-linking.
		 * @detail This sensor's measurements will also be used when
		 * checking for loop-closing. This feature has to be implemented
		 * by the specific sensor.
		 * @param s Name of the other sensor
		 */
		void addLinkSensor(const std::string& s){ mLinkSensors.insert(s); }

		/**
		 * @brief Create a specific measurement from metadata and serialized payload data
		 * @param r robot name
		 * @param s sensor name
		 * @param p sensor pose
		 * @param id uuid of measurement
		 * @param stream serialized payload data
		 * @return pointer to new measurement
		 */
		virtual Measurement::Ptr createFromStream(const std::string& r, const std::string& s,
			const Transform& p, const boost::uuids::uuid id, std::istream& stream)
		{
			throw std::runtime_error("createFromStream() not implemented for " + mName);
		}
		

	protected:
		Mapper* mMapper;
		Logger* mLogger;

		std::string mName;
		IdType mLastVertex; // This is the last vertex from THIS sensor!
		
		float mMinTranslation;
		float mMinRotation;
		
		ScalarType mCovarianceScale;
		std::set<std::string> mLinkSensors;
	};
	
	typedef std::map<std::string, Sensor*> SensorList;
}

#endif
