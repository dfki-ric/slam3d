#pragma once

#include <string>
#include <map>
#include <deque>
#include <mutex>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>


#include <slam3d/core/Types.hpp>
#include <slam3d/core/MeasurementStorage.hpp>

class redisContext;

namespace slam3d {

// uncomment this to enable serialization

class RedisMeasurementStorage: public MeasurementStorage {
 public:

    RedisMeasurementStorage(const char *ip, int port, size_t cacheSize = 0, bool useBinaryArchive = false);

    virtual void add(Measurement::Ptr measurement);

    virtual Measurement::Ptr get(const std::string& key);

    virtual Measurement::Ptr get(const boost::uuids::uuid& key);

    virtual bool contains(const boost::uuids::uuid& key);

    virtual void deleteDatabase();

    /**
     * @brief Set the Split Size redis max value is 512 MB per string
     * 
     * @param splitsize in bytes
     */
    virtual void setSplitSize(const size_t splitsize) {
        split_size = splitsize;
    }

 private:
    void store(const std::string& key, const std::string &type, const std::string& serializedData);

    std::shared_ptr<redisContext> context;
    size_t cacheSize;

    std::deque<std::pair<std::string, Measurement::Ptr>> cache;

    mutable std::mutex queryMutex;

    bool useBinaryArchive;

    size_t split_size;

};



}  // namespace slam3d
