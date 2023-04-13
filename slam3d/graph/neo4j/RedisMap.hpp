#pragma once

#include <string>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>

#include "../../core/Types.hpp"
#include "../../core/MeasurementRegistry.hpp"


namespace slam3d {



class RedisMap {
 public:
    template <class MEASUREMENT_TYPE> void set(const std::string& key, const MEASUREMENT_TYPE &measurement) {
        //measurements[key] = MeasurementRegistry::serialize(measurement);
        measurements[key] = measurement;
    }

    Measurement::Ptr get(const std::string& key);

    template <class MEASUREMENT_TYPE> boost::shared_ptr<MEASUREMENT_TYPE> getAs(const std::string& key) {
        return boost::dynamic_pointer_cast<MEASUREMENT_TYPE>(get(key));
    }

 private:

    // fakedb
    std::map<std::string, Measurement::Ptr> measurements;
    //std::map<std::string, std::string> measurements;

};



}  // namespace slam3d
