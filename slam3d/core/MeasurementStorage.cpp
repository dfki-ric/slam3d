#include "MeasurementStorage.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

using namespace slam3d;

void MeasurementStorage::set(const boost::uuids::uuid& key, Measurement::Ptr measurement)
{
	mMeasurements[key] = measurement;
}

Measurement::Ptr MeasurementStorage::get(const boost::uuids::uuid& uuid)
{
	return mMeasurements.at(uuid);
}

void MeasurementStorage::set(const std::string& key, Measurement::Ptr measurement)
{
	set(boost::lexical_cast<boost::uuids::uuid>(key), measurement);
}

Measurement::Ptr MeasurementStorage::get(const std::string& key)
{
	return get( boost::lexical_cast<boost::uuids::uuid>(key));
}
