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

bool MeasurementSerialization::toFile(Measurement::Ptr measurement, const std::string &filename, bool binary) {
    std::ofstream file(filename);
    if (file.is_open()) {
        if (binary) {
            boost::archive::binary_oarchive oa(file);
            oa << measurement;
        } else {
            boost::archive::text_oarchive oa(file);
            oa << measurement;
        }
        file.close();
        return true;
    }
    return false;
}



Measurement::Ptr MeasurementSerialization::fromFile(const std::string &filename, bool binary) {
    std::ifstream file(filename);
    Measurement::Ptr measurement;
    try {
        if (file.is_open()) {
            if (binary) {
                boost::archive::binary_iarchive ia(file);
                ia >> measurement;
            } else {
                boost::archive::text_iarchive ia(file);
                ia >> measurement;
            }
            file.close();
        }
    } catch (const std::length_error& le) {
        return fromFile(filename, !binary);
    } catch (const boost::archive::archive_exception ae) {
        return fromFile(filename, !binary);
    }
    return measurement;
}

void MeasurementSerialization::toDirectory(MeasurementStorage* storage, const std::string &dir, bool binary)
{
	for(const auto& entry : *storage)
	{
		const std::string filename(dir + "/" + boost::lexical_cast<std::string>(entry.second->getUniqueId()) + ".s3dm");
		toFile(entry.second, filename, binary);
	}
}

void MeasurementSerialization::fromDirectory(MeasurementStorage* storage, const std::string &dir, bool binary)
{
	for(const auto& entry : std::filesystem::directory_iterator{dir}) 
	{
		if(entry.path().extension() == ".s3dm")
		{
			storage->add(fromFile(entry.path(), binary));
		}
	}
}

std::string MeasurementSerialization::toString(Measurement::Ptr measurement, bool binary) {
    std::ostringstream ss;
    if (binary) {
        boost::archive::binary_oarchive oa(ss);
        oa << measurement;
    } else {
        boost::archive::text_oarchive oa(ss);
        oa << measurement;
    }
    return ss.str();
}

Measurement::Ptr MeasurementSerialization::fromString(const std::string &serialized, bool binary) {
    Measurement::Ptr measurement;
    std::stringstream serializedData(serialized);
    try {
        if (binary) {
            boost::archive::binary_iarchive ia(serializedData);
            ia >> measurement;
        } else {
            boost::archive::text_iarchive ia(serializedData);
            ia >> measurement;
        }
    } catch (const std::length_error& le) {
        return fromString(serialized, !binary);
    } catch (const boost::archive::archive_exception ae) {
        return fromString(serialized, !binary);
    }
    return measurement;
}
