#pragma once

#include <string>
#include <map>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <slam3d/core/Types.hpp>
#include <slam3d/core/MeasurementStorage.hpp>

class redisContext;

namespace slam3d {

// uncomment this to enable serialization

class RedisMeasurementStorage: public MeasurementStorage {
 public:

    RedisMeasurementStorage(const char *ip, int port);

    virtual void set(const std::string& key, Measurement::Ptr measurement);

    virtual Measurement::Ptr get(const std::string& key);


 private:
    void store(const std::string& key, const std::string &type, const std::string& serializedData);

    std::shared_ptr<redisContext> context;

};



}  // namespace slam3d
