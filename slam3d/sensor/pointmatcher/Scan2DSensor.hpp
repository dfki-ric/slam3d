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

#ifndef SLAM_SCANSENSOR_HPP
#define SLAM_SCANSENSOR_HPP

#include <slam3d/core/Mapper.hpp>
#include <slam3d/core/ScanSensor.hpp>

#include <pointmatcher/PointMatcher.h>

namespace slam3d
{
	typedef PointMatcher<ScalarType> PM;

	/**
	 * @class Scan2DMeasurement
	 * @brief 
	 */
	class Scan2DMeasurement : public Measurement
	{
	public:
		typedef boost::shared_ptr<Scan2DMeasurement> Ptr;

		Scan2DMeasurement(const PM::DataPoints& points, timeval t,
	                    const std::string& r, const std::string& s,
	                    const Transform& p, const boost::uuids::uuid id = boost::uuids::nil_uuid())
		: Measurement(r, s, p, id), mDataPoints(points) { mStamp = t; }

		const PM::DataPoints& getDataPoints() { return mDataPoints; }

	protected:
		PM::DataPoints mDataPoints;
	};

	/**
	 * @class Scan2DSensor
	 * @brief 
	 */
	class Scan2DSensor : public ScanSensor
	{
	public:
		/**
		 * @brief Constructor
		 * @param n unique name of this sensor (used to identify measurements)
		 * @param l pointer to a Logger to write messages
		 * @param c path to ICP configuration file
		 */
		Scan2DSensor(const std::string& n, Logger* l, const std::string& c);

		/**
		 * @brief Destructor
		 */
		~Scan2DSensor();

		/**
		 * @brief Create a linking constraint between source and target.
		 * @param source_id
		 * @param target_id
		 */
		void link(IdType source_id, IdType target_id);

		/**
		 * @brief Calculate the estimated transform between two measurements of this sensor.
		 * @details It uses the ICP class from libpointmatcher.
		 * @param source measurement of the source node
		 * @param target measurement of the target node
		 * @param odometry estimation of robot movement
		 * @param debug whether to write alignment result to file
		 * @throw BadMeasurementType
		 */
		TransformWithCovariance calculateTransform(Measurement::Ptr source,
		                                           Measurement::Ptr target,
		                                           TransformWithCovariance odometry,
		                                           bool debug = false);

		/**
		 * @brief Create a virtual measurement by accumulating pointclouds from given vertices.
		 * @param vertices list of vertices that should contain a PointCloudMeasurement
		 * @param pose origin of the accumulated pointcloud
		 * @throw BadMeasurementType
		 */		
		Measurement::Ptr createCombinedMeasurement(const VertexObjectList& vertices, Transform pose) const;

		/**
		 * @brief Convert the 2D ICP result to a 3D transformation
		 * @param in
		 */
		Transform convert2Dto3D(const PM::TransformationParameters& in) const;

		/**
		 * @brief Convert a 3D transformation to a 2D transformation (x,y,yaw)
		 * @param in
		 */
		PM::TransformationParameters convert3Dto2D(const Transform& in) const;

		/**
		 * @brief Creates an empty DataPoints struct for 2D scans (x,y,w)
		 */
		PM::DataPoints createDataPoints() const;

		/**
		 * @brief Transforms the source by the given transform.
		 * @param source
		 * @param tf
		 */
		PM::DataPoints transformDataPoints(const PM::DataPoints& source, const Transform tf) const;

		/**
		 * @brief Activate writing of source and target pointclouds when perfomring loop closures.
		 * @param debug
		 */
		void writeDebugData(bool debug = true) { mWriteDebugData = debug; }

	protected:
		PM::ICP mICP;

		bool mWriteDebugData; 
	};
}

#endif
