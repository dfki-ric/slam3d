# pragma once

#include <string>
#include <vector>
#include <memory>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "Types.hpp"

namespace slam3d {

class MeasurementToStringBase {
 public:
    virtual ~MeasurementToStringBase() {}
    virtual std::string serialize(Measurement::Ptr ptr) = 0;
    virtual Measurement::Ptr deserialize(const std::string &data) = 0;
    virtual bool isSameType(Measurement::Ptr ptr) = 0;


};

template <class MEASUREMENT_TYPE> class MeasurementToString : public MeasurementToStringBase {
 public:
    MeasurementToString(int registry_index) : registry_index(registry_index) {}
    virtual ~MeasurementToString() {}

    virtual bool isSameType(Measurement::Ptr ptr) {
        boost::shared_ptr<MEASUREMENT_TYPE> newptr = boost::dynamic_pointer_cast<MEASUREMENT_TYPE>(ptr);
        if (newptr.get()) {
            return true;
        }
        return false;
    }

    virtual std::string serialize(Measurement::Ptr ptr) {
        boost::shared_ptr<MEASUREMENT_TYPE> newptr = boost::dynamic_pointer_cast<MEASUREMENT_TYPE>(ptr);
        ptr->setRegistryIndex(registry_index);
        printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);

        std::stringstream ss;
        boost::archive::text_oarchive oa(ss);
        oa << *(newptr.get());
        return ss.str();
    }

    virtual Measurement::Ptr deserialize(const std::string &data) {
        boost::shared_ptr<MEASUREMENT_TYPE> m = boost::make_shared<MEASUREMENT_TYPE>();
        std::stringstream ss(data);
        boost::archive::text_iarchive ia(ss);
        ia >> *(m.get());
        return m;
    }

 private:
    int registry_index;
};


class MeasurementRegistry {
 public:
    template <class TYPE> static void registerMeasurementType() {
        std::shared_ptr<MeasurementToStringBase> conv = std::make_shared<MeasurementToString<TYPE>>(converters.size());
        converters.push_back(conv);
    }

    static size_t getIdOf(Measurement::Ptr ptr) {
        for (int i = 0; i < converters.size(); ++i) {
            if (converters[i]->isSameType(ptr)) {
                printf("converter type %i\n", i);
                return i;
            }
        }
        return -1;
    }

    static std::string serialize(Measurement::Ptr ptr) {
        return converters[getIdOf(ptr)]->serialize(ptr);
    }

    static Measurement::Ptr deserialize(const std::string &data) {

        //deserialize base
        Measurement m("","", slam3d::Transform());
        std::stringstream ss(data);
        boost::archive::text_iarchive ia(ss);
        ia >> m;
        //deserialize full
        return converters[m.getRegistryIndex()]->deserialize(data);
    }

 private:
    static std::vector< std::shared_ptr<MeasurementToStringBase> > converters;
};

}  // namespace slam3d
