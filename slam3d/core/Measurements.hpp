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
#include <boost/lexical_cast.hpp>

namespace slam3d {

class Measurements {
 public:
    Measurements() {}
    virtual ~Measurements() {}

    virtual void set(const boost::uuids::uuid& key, Measurement::Ptr measurement);
    virtual Measurement::Ptr get(const boost::uuids::uuid& key);

    void set(const std::string& key, Measurement::Ptr measurement) {
        set(boost::lexical_cast<boost::uuids::uuid>(key), measurement);
    }

    Measurement::Ptr get(const std::string& key) {
        return get( boost::lexical_cast<boost::uuids::uuid>(key));
    }

 private:
    std::map<boost::uuids::uuid, Measurement::Ptr> measurements;
};


}  // namespace slam3d