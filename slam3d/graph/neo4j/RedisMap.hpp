#pragma once

#include <string>
#include <map>
#include <boost/archive/text_oarchive.hpp>

#include "../../core/Types.hpp"
#include "../../core/MeasurementRegistry.hpp"


namespace slam3d {

// uncomment this to enable serialization
//#define SERIALIZE

class RedisMap {
 public:
    template <class MEASUREMENT_TYPE> void set(const std::string& key, const MEASUREMENT_TYPE &measurement) {
        #ifdef SERIALIZE
            measurements[key] = MeasurementRegistry::serialize(measurement);
            typenames[key] = measurement->getMeasurementTypeName();
        #else
            measurements[key] = measurement;
        #endif
    }

    Measurement::Ptr get(const std::string& key);


 private:
    #ifdef SERIALIZE
        std::map<std::string, std::string> typenames;
        std::map<std::string, std::string> measurements;
    #else
        std::map<std::string, Measurement::Ptr> measurements;
    #endif

};



}  // namespace slam3d
