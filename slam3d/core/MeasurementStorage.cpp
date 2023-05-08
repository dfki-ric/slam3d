#include "MeasurementStorage.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

using namespace slam3d;

void MeasurementStorage::add(Measurement::Ptr measurement)
{
	mMeasurements[measurement->getUniqueId()] = measurement;
}

Measurement::Ptr MeasurementStorage::get(const boost::uuids::uuid& uuid)
{
	return mMeasurements.at(uuid);
}

Measurement::Ptr MeasurementStorage::get(const std::string& key)
{
	return get( boost::lexical_cast<boost::uuids::uuid>(key));
}
