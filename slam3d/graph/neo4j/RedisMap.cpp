#include <hiredis/hiredis.h>

#include "RedisMap.hpp"

#include <boost/archive/text_iarchive.hpp>


namespace slam3d {

    Measurement::Ptr RedisMap::get(const std::string& key) {
        #ifdef SERIALIZE
            return MeasurementRegistry::deserialize(measurements[key], typenames[key]);
        #else
            return measurements[key];
        #endif
    }

}  // namespace slam3d
