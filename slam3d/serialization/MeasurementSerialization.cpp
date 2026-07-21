#include "MeasurementSerialization.hpp"
#include "../sensor/pcl/PointCloudSensor.hpp"

#include <fstream>
#include <sstream>
#include <filesystem>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/lexical_cast.hpp>

BOOST_CLASS_EXPORT_IMPLEMENT(slam3d::PointCloudMeasurement)

using namespace slam3d;

bool MeasurementSerialization::toFile(Measurement::Ptr measurement, const std::string &filename, bool binary)
{
	std::ofstream file(filename);
	if (file.is_open())
	{
		if (binary)
		{
			boost::archive::binary_oarchive oa(file);
			oa << measurement;
		}else
		{
			boost::archive::text_oarchive oa(file);
			oa << measurement;
		}
		file.close();
		return true;
	}
	return false;
}

Measurement::Ptr MeasurementSerialization::fromFile(const std::string &filename, bool binary)
{
	std::ifstream file(filename);
	Measurement::Ptr measurement;
	if (file.is_open())
	{
		if (binary)
		{
			boost::archive::binary_iarchive ia(file);
			ia >> measurement;
		}else
		{
			boost::archive::text_iarchive ia(file);
			ia >> measurement;
		}
		file.close();
	}
	return measurement;
}

void MeasurementSerialization::toDirectory(MeasurementStorage* storage, const std::string &dir, bool binary)
{
	for(const auto& uuid : storage->getAllKeys())
	{
		const std::string filename(dir + "/" + boost::lexical_cast<std::string>(uuid) + ".s3dm");
		toFile(storage->get(uuid), filename, binary);
	}
}

void MeasurementSerialization::fromDirectory(MeasurementStorage* storage, const std::string &dir, bool binary)
{
	for(const auto& entry : std::filesystem::directory_iterator{dir}) 
	{
		if(entry.path().extension() == ".s3dm")
		{
			const boost::uuids::uuid id = boost::lexical_cast<boost::uuids::uuid>(entry.path().stem());
			storage->add(fromFile(entry.path(), binary), id);
		}
	}
}

std::string MeasurementSerialization::toString(Measurement::Ptr measurement, bool binary)
{
	std::ostringstream ss;
	if (binary)
	{
		boost::archive::binary_oarchive oa(ss);
		oa << measurement;
	}else
	{
		boost::archive::text_oarchive oa(ss);
		oa << measurement;
	}
	return ss.str();
}

Measurement::Ptr MeasurementSerialization::fromString(const std::string &serialized, bool binary)
{
	Measurement::Ptr measurement;
	std::stringstream serializedData(serialized);
	if (binary)
	{
		boost::archive::binary_iarchive ia(serializedData);
		ia >> measurement;
	}else
	{
		boost::archive::text_iarchive ia(serializedData);
		ia >> measurement;
	}
	return measurement;
}
