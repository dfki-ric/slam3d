
#include "Measurements.hpp"

namespace slam3d {

    void Measurements::set(const std::string& key, Measurement::Ptr measurement) {
        measurements[key] = measurement;
    }

    Measurement::Ptr Measurements::get(const std::string& key) {
        return measurements.at(key);
    }

}  // namespace slam3d