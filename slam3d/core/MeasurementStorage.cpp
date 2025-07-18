#include "MeasurementStorage.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

using namespace slam3d;

void MeasurementStorage::add(Measurement::Ptr measurement)
{
	if (enabled) {
		mMeasurements[measurement->getUniqueId()] = measurement;
	}

}

Measurement::Ptr MeasurementStorage::get(const boost::uuids::uuid& uuid)
{
	try {
		return mMeasurements.at(uuid);
	}catch (const std::out_of_range& e) {
		return Measurement::Ptr();
	}
}

Measurement::Ptr MeasurementStorage::get(const std::string& key)
{
	return get( boost::lexical_cast<boost::uuids::uuid>(key));
}

bool MeasurementStorage::contains(const boost::uuids::uuid& key)
{
	return mMeasurements.count(key);
}
		
void MeasurementStorage::enable()
{
	enabled = true;
}

void MeasurementStorage::disable()
{
	enabled = false;
}
