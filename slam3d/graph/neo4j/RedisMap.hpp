#pragma once

#include <string>
#include <map>
#include <boost/archive/text_oarchive.hpp>

#include "../../core/Types.hpp"
#include "../../core/MeasurementRegistry.hpp"

class redisContext;

namespace slam3d {

// uncomment this to enable serialization
// #define SERIALIZE

class RedisMap {
 public:

    RedisMap(const char *ip, int port);

    template <class MEASUREMENT_TYPE> void set(const std::string& key, const MEASUREMENT_TYPE &measurement) {
        store(key, measurement->getMeasurementTypeName(), MeasurementRegistry::serialize(measurement));
    }

    Measurement::Ptr get(const std::string& key);


 private:

    void store(const std::string& key, const std::string &type, const std::string& serializedData);


    // #ifdef SERIALIZE
    //     std::map<std::string, std::string> typenames;
    //     std::map<std::string, std::string> measurements;
    // #else
    //     std::map<std::string, Measurement::Ptr> measurements;
    // #endif

    std::shared_ptr<redisContext> context;

};



}  // namespace slam3d
