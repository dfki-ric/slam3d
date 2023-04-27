
#include "Measurements.hpp"

namespace slam3d {

    void Measurements::set(const boost::uuids::uuid& key, Measurement::Ptr measurement) {
        measurements[key] = measurement;
    }

    Measurement::Ptr Measurements::get(const boost::uuids::uuid& uuid) {
        return measurements.at(uuid);
    }

}  // namespace slam3d