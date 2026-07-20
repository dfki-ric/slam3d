#include "MeasurementStorage.hpp"

#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

using namespace slam3d;

void MeasurementStorage::add(Measurement::Ptr measurement, const boost::uuids::uuid& uuid)
{
	if(uuid.is_nil())
	{
		throw std::runtime_error("Measurement added to storage has nil-uuid.");
	}
	mMeasurements[uuid] = measurement;
}

Measurement::Ptr MeasurementStorage::get(const boost::uuids::uuid& uuid)
{
	if(uuid.is_nil())
		return {};
	else
		return mMeasurements.at(uuid);
}

Measurement::Ptr MeasurementStorage::get(const std::string& key)
{
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

std::vector<boost::uuids::uuid> MeasurementStorage::getAllKeys() const
{
	std::vector<boost::uuids::uuid> result;
	for(const auto& m : mMeasurements)
	{
		result.push_back(m.first);
	}
	return result;
}
