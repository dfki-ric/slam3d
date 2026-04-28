#include "MeasurementSerialization.hpp"


#include "../sensor/pcl/PointCloudSensor.hpp"
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


