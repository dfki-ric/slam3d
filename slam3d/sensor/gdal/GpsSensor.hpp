// slam3d - Frontend for graph-based SLAM
// Copyright (C) 2019 S. Kasperski
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

#ifndef SLAM3D_GPSSENSOR_HPP
#define SLAM3D_GPSSENSOR_HPP

#include <slam3d/core/Sensor.hpp>

class OGRCoordinateTransformation;

namespace slam3d
{
	typedef Eigen::Matrix<ScalarType,3,1> Position; // Eventually move to Types.hpp
	
	class GpsMeasurement : public Measurement
	{
	public:
		typedef boost::shared_ptr<GpsMeasurement> Ptr;
		
		GpsMeasurement( timeval t, const std::string& r, const std::string& s,
	                    const Transform& p, const boost::uuids::uuid id = boost::uuids::nil_uuid())
		: Measurement(r, s, p, id) { mStamp = t; }
		
		~GpsMeasurement() {}
		
	protected:
		
	};
	
	class GpsSensor : public Sensor
	{
	public:
		GpsSensor(const std::string& n, Logger* l) : Sensor(n, l) {}
		~GpsSensor() {}
		
		void initCoordTransform(int utmZone = 32, bool utmNorth = true);
		void addMeasurement(const GpsMeasurement::Ptr&m);
		
	protected:
		OGRCoordinateTransformation* mCoordTransform;
	
		int mUtmZone;
		bool mUtmNorth;
		Position mUtmOrigin;
	};
}

#endif
