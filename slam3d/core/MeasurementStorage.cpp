#include "MeasurementStorage.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

using namespace slam3d;

void MeasurementStorage::add(Measurement::Ptr measurement)
{
	if(measurement->getUniqueId().is_nil())
	{
		throw std::runtime_error("Measurement added to storage has nil-uuid.");
	}
	printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__, boost::lexical_cast<std::string>(measurement->getUniqueId()).c_str());
	mMeasurements[measurement->getUniqueId()] = measurement;
}

Measurement::Ptr MeasurementStorage::get(const boost::uuids::uuid& uuid)
{
	printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__, boost::lexical_cast<std::string>(uuid).c_str());
	if(uuid.is_nil())
		return {};
	else
		return mMeasurements.at(uuid);
}

Measurement::Ptr MeasurementStorage::get(const std::string& key)
{
	printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__, key.c_str());
	return get( boost::lexical_cast<boost::uuids::uuid>(key));
}

bool MeasurementStorage::contains(const boost::uuids::uuid& key)
{
	return mMeasurements.count(key);
}

void MeasurementStorage::clear()
{
	mMeasurements.clear();
}
