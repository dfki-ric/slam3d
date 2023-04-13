#include <hiredis/hiredis.h>

#include "RedisMap.hpp"


#include <boost/archive/text_iarchive.hpp>


namespace slam3d {

// Measurement::Ptr& RedisMap::operator[](const std::string& key) {
//     return measurements[key];
// }


    Measurement::Ptr RedisMap::get(const std::string& key) {
        //return MeasurementRegistry::deserialize(measurements[key]);
        return measurements[key];
    }

}  // namespace slam3d
