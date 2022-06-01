#include "CoordTransformer.hpp"

#include <ogr_spatialref.h>

using namespace slam3d;

void CoordTransformer::init(int utmZone, bool utmNorth)
{
	OGRSpatialReference source;
	OGRSpatialReference target;
	
	source.SetWellKnownGeogCS("WGS84");
	target.SetWellKnownGeogCS("WGS84");
	target.SetUTM(utmZone, utmNorth);
	
	mCoordTransform = OGRCreateCoordinateTransformation(&source, &target);
	
	if(!mCoordTransform)
	{
		throw std::runtime_error("Failed to initialize coordinate transformation.");
	}
}

Position CoordTransformer::toUTM(ScalarType lon, ScalarType lat, ScalarType alt)
{
	if(!mCoordTransform)
	{
		throw std::runtime_error("You must call CoordTransformer::init before using toUTM()!");
	}

	if(!mCoordTransform->Transform(1, &lon, &lat, &alt))
	{
		throw std::runtime_error("Transformation failed!");
	}
	Position utm;
	utm(0) = lon;
	utm(1) = lat;
	utm(2) = alt;
	return utm - mReference;
}

void CoordTransformer::setReference(ScalarType lon, ScalarType lat, ScalarType alt)
{
	mReference = toUTM(lon, lat, alt);
}
