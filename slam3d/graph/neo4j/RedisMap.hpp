#pragma once

#include <string>
#include <map>
#include <boost/archive/text_oarchive.hpp>

#include <slam3d/core/Types.hpp>
#include <slam3d/core/MeasurementSerialization.hpp>
#include <slam3d/core/Measurements.hpp>

class redisContext;

namespace slam3d {

// uncomment this to enable serialization

class RedisMap: public Measurements {
 public:

    RedisMap(const char *ip, int port);

    virtual void set(const std::string& key, Measurement::Ptr measurement) {
        store(key, measurement->getMeasurementTypeName(), MeasurementSerialization::serialize(measurement));
    }

    // template <class MEASUREMENT_TYPE> void set(const std::string& key, const MEASUREMENT_TYPE &measurement) {
    //     
    // }

    virtual Measurement::Ptr get(const std::string& key);


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
