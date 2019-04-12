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
#include <pointmatcher/PointMatcher.h>

namespace slam3d
{
	typedef PointMatcher<ScalarType> PM;

	/**
	 * @class ScanMeasurement
	 * @brief 
	 */
	class ScanMeasurement : public Measurement
	{
	public:
		typedef boost::shared_ptr<ScanMeasurement> Ptr;
		
		ScanMeasurement(const PM::DataPoints& points, timeval t,
	                    const std::string& r, const std::string& s,
	                    const Transform& p, const boost::uuids::uuid id = boost::uuids::nil_uuid())
		: Measurement(r, s, p, id), mDataPoints(points) { mStamp = t; }

		const PM::DataPoints& getDataPoints() { return mDataPoints; }

	protected:
		PM::DataPoints mDataPoints;
	};

	/**
	 * @class ScanSensor
	 * @brief 
	 */
	class ScanSensor : public Sensor
	{
	public:
		/**
		 * @brief Constructor
		 * @param n unique name of this sensor (used to identify measurements)
		 * @param l pointer to a Logger to write messages
		 */
		ScanSensor(const std::string& n, Logger* l);
		
		/**
		 * @brief Destructor
		 */
		~ScanSensor();
		
		bool addMeasurement(const ScanMeasurement::Ptr& scan, const Transform& odom);
		
		TransformWithCovariance calculateTransform(ScanMeasurement::Ptr source,
		                                           ScanMeasurement::Ptr target,
		                                           TransformWithCovariance odometry);

		Transform convert2Dto3D(const PM::TransformationParameters& in);
		PM::TransformationParameters convert3Dto2D(const Transform& in);

	protected:
		PM::ICP mICP;

		Transform mLastOdometry;
		TransformWithCovariance mOdometryDelta;
	};
}

#endif
