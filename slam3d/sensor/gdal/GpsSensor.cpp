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

#include "GpsSensor.hpp"

#include <slam3d/core/Mapper.hpp>

#include <ogr_spatialref.h>
#include <boost/format.hpp>

using namespace slam3d;

void GpsSensor::initCoordTransform(int utmZone, bool utmNorth)
{
	OGRSpatialReference source;
	OGRSpatialReference target;
	
	source.SetWellKnownGeogCS("WGS84");
	target.SetWellKnownGeogCS("WGS84");
	target.SetUTM(utmZone, utmNorth);
	
	mCoordTransform = OGRCreateCoordinateTransformation(&source, &target);
	
	if(!mCoordTransform)
	{
		throw std::runtime_error("GpsSensor failed to initialize coordinate transformation.");
	}
}

Position GpsSensor::toUTM(ScalarType lon, ScalarType lat, ScalarType alt)
{
	mCoordTransform->Transform(1, &lon, &lat, &alt);
	Position utm;
	utm(0) = lon;
	utm(1) = lat;
	utm(2) = alt;
	return utm;
}

void GpsSensor::addMeasurement(const GpsMeasurement::Ptr &m)
{
	if(!mLastVertex)
	{
		mReference = m->getPosition();
	}else
	{
		Position delta = m->getPosition() - mLastPosition;
		if(delta.norm() < mMinTranslation)
			return;
	}

	mLastVertex = mMapper->addMeasurement(m);
	Position rel_pos = m->getPosition() - mReference;
	mLogger->message(DEBUG, (boost::format("GPS: relative pose (%1%, %2%, %3%)") % rel_pos(0) % rel_pos(1) % rel_pos(2)).str());
	PositionConstraint::Ptr position(new PositionConstraint(mName, rel_pos, m->getCovariance()));
	mMapper->getGraph()->addConstraint(mLastVertex, 0, position);
	mLastPosition = m->getPosition();
}
