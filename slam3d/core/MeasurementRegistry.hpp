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

class MeasurementToStringBase {
 public:
    virtual ~MeasurementToStringBase() {}
    virtual std::string serialize(Measurement::Ptr ptr) = 0;
    virtual Measurement::Ptr deserialize(const std::string &data) = 0;
    virtual bool isSameType(Measurement::Ptr ptr) = 0;
    virtual const std::string& getTypeName() = 0;
};

template <class MEASUREMENT_TYPE> class MeasurementToString : public MeasurementToStringBase {
 public:
    explicit MeasurementToString(const std::string& typeName) {
        measurementTypeName = typeName;
    }
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

    virtual const std::string& getTypeName() {
        return measurementTypeName;
    }

 private:
    std::string measurementTypeName;
};


class MeasurementRegistry {
 public:
    /**
     * @brief register a (de-)serializer fo a given type
     * 
     * @tparam TYPE The measuremetn type that should be (de-)serialized
     * @param measurementTypeName the type name that is returned when TYPE.getMeasurementTypeName() is called. This si used to match the fitting serializer
     */
    template <class TYPE> static void registerMeasurementType(const std::string& measurementTypeName) {
        std::shared_ptr<MeasurementToStringBase> conv = std::make_shared< MeasurementToString<TYPE> >(measurementTypeName);
        converters[measurementTypeName] = conv;
    }


    static std::string serialize(Measurement::Ptr ptr) {
        if (ptr.get()) {
            std::shared_ptr<MeasurementToStringBase> conv = converters[ptr->getMeasurementTypeName()];
            if (conv.get()) {
                return conv->serialize(ptr);
            } else {
                printf("no registered converter for %s\n", ptr->getMeasurementTypeName().c_str());
            }
        }
        return "";
    }

    static Measurement::Ptr deserialize(const std::string &data, const std::string& key) {
        if (data.size()) {
            std::shared_ptr<MeasurementToStringBase> conv = converters[key];
            if (conv.get()) {
                return converters[key]->deserialize(data);
            } else {
                printf("no registered converter for %s\n", key.c_str());
            }
            return Measurement::Ptr();
        }
        return Measurement::Ptr();
    }

 private:
    static std::map<std::string, std::shared_ptr<MeasurementToStringBase> > converters;
};

}  // namespace slam3d
