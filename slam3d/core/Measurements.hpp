# pragma once

#include <string>
#include <map>
#include <memory>

// #include <typeinfo>
// #include <cxxabi.h>


#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "Types.hpp"
#include <boost/uuid/uuid_io.hpp>

namespace slam3d {

class Measurements {
 public:
    Measurements() {}
    virtual ~Measurements() {}

    void set(const boost::uuids::uuid& uuid, Measurement::Ptr measurement) {
        set(boost::uuids::to_string(uuid), measurement);
    }

    virtual void set(const std::string& key, Measurement::Ptr measurement);

    Measurement::Ptr get(const boost::uuids::uuid& uuid) {
        return get(boost::uuids::to_string(uuid));
    }

    virtual Measurement::Ptr get(const std::string& key);


 private:
    std::map<std::string, Measurement::Ptr> measurements;


};


}  // namespace slam3d