
#include "MeasurementStorage.hpp"

namespace slam3d {

    void MeasurementStorage::set(const boost::uuids::uuid& key, Measurement::Ptr measurement) {
        measurements[key] = measurement;
    }

    Measurement::Ptr MeasurementStorage::get(const boost::uuids::uuid& uuid) {
        return measurements.at(uuid);
    }

}  // namespace slam3d