# pragma once

#include <string>
#include <map>
#include <memory>

// #include <typeinfo>
// #include <cxxabi.h>


#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "Types.hpp"

namespace slam3d {

class Measurements {
 public:
    Measurements() {}
    virtual ~Measurements() {}

    virtual void set(const std::string& key, Measurement::Ptr measurement);

    virtual Measurement::Ptr get(const std::string& key);


 private:
    std::map<std::string, Measurement::Ptr> measurements;


};


}  // namespace slam3d