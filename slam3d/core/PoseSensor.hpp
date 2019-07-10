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

#ifndef SLAM_POSESENSOR_HPP
#define SLAM_POSESENSOR_HPP

#include "Types.hpp"
#include "Logger.hpp"

namespace slam3d
{
	class Graph;
	
	class InvalidPose : public std::exception
	{
	public:
		InvalidPose(const std::string& r)
		: reason(r) {}
		~InvalidPose() throw() {}
		
		virtual const char* what() const throw()
		{
			return reason.c_str();
		}
		
		std::string reason;
	};
	
/**
 * @class PoseSensor
 * @brief Abstract base class for all pose providing sensors.
 * @details PoseSensors provide relative or absolute poses which are used
 * to connect measurements of the used Sensor's. Unlike the latter, which
 * create new nodes in the graph, PoseSensor's only create edges. Upon
 * creating a new vertex, handleNewVertex will be called on each registered
 * PoseSensor with the new Vertex' ID. Examples for a PoseSensor are Odometry,
 * Inertial Measurement Units, tracking systems or GPS.
 */
	class PoseSensor
	{
	public:
		// Ctor / Dtor
		PoseSensor(const std::string& n, Graph* g, Logger* l) : mGraph(g), mLogger(l), mName(n), mCovarianceScale(1.0) {}
		virtual ~PoseSensor(){};
		
		// Virtual methods
		/**
		 * @brief Process a newly added vertex.
		 * @details Main callback method that is called for each vertex
		 * that is added to the graph. The specific implementation might add
		 * a new edge to the previous node or some global reference node.
		 * @param vertex ID of the newly added vertex.
		 */
		virtual void handleNewVertex(IdType vertex) = 0;

		/**
		 * @brief Get the pose of the PoseSensor at the given time in its
		 * own reference frame. (e.g. odometry coordinates)
		 * @param stamp
		 */
		virtual Transform getPose(timeval stamp) = 0;

		// Access methods
		/**
		 * @brief Get this sensor's registered name.
		 * @return Name of the sensor.
		 */
		std::string getName(){ return mName; }
	
		/**
		 * @brief Set the covariance scale for measurements of this sensor.
		 * @param s
		 */
		void setCovarianceScale(ScalarType s){ mCovarianceScale = s; }
	
	protected:
		Graph* mGraph;
		Logger* mLogger;
		
		std::string mName;
		ScalarType mCovarianceScale;
	};
	
	typedef std::map<std::string, PoseSensor*> PoseSensorList;
}

#endif
