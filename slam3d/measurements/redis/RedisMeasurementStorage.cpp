#include <hiredis/hiredis.h>
#include <boost/serialization/shared_ptr.hpp>

#include "RedisMeasurementStorage.hpp"


namespace slam3d {

    RedisMeasurementStorage::RedisMeasurementStorage(const char *ip, int port, size_t cacheSize, bool useBinaryArchive):MeasurementStorage(),cacheSize(cacheSize),useBinaryArchive(useBinaryArchive), split_size(500*1024*1024) {
        context = std::shared_ptr<redisContext>(redisConnect(ip, port));

        if (context.get() == nullptr || context->err) {
            if (context.get()) {
                printf("Error: %s\n", context->errstr);
                // handle error
            } else {
                printf("Can't allocate redis context\n");
            }
        }
        //redisCommand(context.get(), "flushdb");
    }

    void RedisMeasurementStorage::add(Measurement::Ptr measurement) {
        if (enabled) {
            if (useBinaryArchive) {
                std::ostringstream ss;
                boost::archive::binary_oarchive oa(ss);
                oa << measurement;
                store(to_string(measurement->getUniqueId()), measurement->getTypeName(), ss.str());
            } else {
                std::stringstream ss;
                boost::archive::text_oarchive oa(ss);
                oa << measurement;
                store(to_string(measurement->getUniqueId()), measurement->getTypeName(), ss.str());
            }
        }
    }

    void RedisMeasurementStorage::store(const std::string& key, const std::string &type, const std::string& serializedData) {
        // printf("HSET %s type %s, data %s\n", key.c_str(), type.c_str(), "data");
        std::lock_guard<std::mutex> lock(queryMutex);

        // split large data into different slices
        // const size_t split_size = (256*1024*1024); // 256MB per entry
        size_t elements = (serializedData.size()/split_size); // +1 to accomondate the rest

        if (serializedData.size() > split_size) {
            void* reply = redisCommand(context.get(), "HSET %s size %i type %s", key.c_str(), elements, type.c_str());
            if (!reply) {
                printf("coud not store measurement: %s\n", context->errstr);
                // TODO: recreate context
                return;
            }
            freeReplyObject(reply);
        }

        size_t chars_to_save = serializedData.size();
        size_t chars_saved = 0;
        for (size_t i = 0; i <= elements; ++i) {
            
            std::string cmd = "HSET %s data" + std::to_string(i) + " %b";

            //std::string cmd = "HSET %s type %s data %b";
            size_t part_size = (chars_to_save > split_size) ? split_size : serializedData.size()-chars_saved;

            void* reply = redisCommand(context.get(), cmd.c_str(), key.c_str(), serializedData.data() + chars_saved , part_size);
            if (!reply) {
                printf("coud not store measurement: %s\n", context->errstr);
                // TODO: recreate context
                return;
            }
            chars_to_save -= split_size;
            chars_saved += part_size;
            freeReplyObject(reply);
        }        
    }

    Measurement::Ptr RedisMeasurementStorage::get(const std::string& key) {
        // check cache
        if (cacheSize > 0) {
            auto uuidIterator = std::find_if(cache.begin(),cache.end(),[&key](const std::pair<std::string, Measurement::Ptr>  &pair){
                                                return pair.first == key;
                                            });
            if (uuidIterator != cache.end()) {
                // if in cache, return here
                return uuidIterator->second;
            }
        }

        // read type and size
        std::lock_guard<std::mutex> lock(queryMutex);
        
        size_t elements = 0;

    
        void* metareply = redisCommand(context.get(), "HGET %s size", key.c_str());
        if (!metareply) {
            printf("coud not load measurement: %s\n", context->errstr);
            // TODO: recreate context
            freeReplyObject(metareply);
            throw std::out_of_range("not found in database");
        }
        redisReply* redismetarep = (redisReply*)metareply;
        if (redismetarep->str == nullptr) {
            // older saved data, where size field is not set
            elements = 1; 
        } else {
            elements = std::stoi(redismetarep->str);
        }
        freeReplyObject(metareply);


        std::string cmd = "HMGET %s ";
        for (size_t i = 0; i<=elements;++i) {
            cmd += "data" + std::to_string(i) + " ";
        }

        void* reply = redisCommand(context.get(), cmd.c_str(), key.c_str());
        if (!reply) {
            printf("coud not load measurement: %s\n", context->errstr);
            // TODO: recreate context
            freeReplyObject(reply);
            throw std::out_of_range("not found in database");
        }
        redisReply* redisrep = (redisReply*)reply;
        
        Measurement::Ptr measurement;
        if (redisrep->elements > 0) {
            // assemble 
            std::stringstream serializedData;

            for (size_t elem = 0; elem < redisrep->elements; ++elem) {

                serializedData << std::string(redisrep->element[elem]->str, redisrep->element[elem]->len);
            }

            if (useBinaryArchive) {
                try {
                    boost::archive::binary_iarchive ia(serializedData);
                    ia >> measurement;
                } catch (const std::length_error& e) {
                    printf("could not read measurement, try setting the boost archive type to text\n");
                }
            } else {
                
                try {
                    boost::archive::text_iarchive ia(serializedData);
                    ia >> measurement;
                } catch (const std::length_error& e) {
                    printf("could not read measurement, try setting the boost archive type to binary\n");
                }
            }
            if (cacheSize > 0) {
                while (cache.size() >= cacheSize){
                    cache.pop_front();
                }
                cache.push_back({to_string(measurement->getUniqueId()) , measurement});
            }
            
        } else {
            printf("%s:%i got empty measurement reply\n",__PRETTY_FUNCTION__,__LINE__);
        }

        //todo: create from 

        // redisReaderFree(reader);
        freeReplyObject(reply);
        return measurement;
    }

    Measurement::Ptr RedisMeasurementStorage::get(const boost::uuids::uuid& key) {
        return get(to_string(key));
    }

    bool RedisMeasurementStorage::contains(const boost::uuids::uuid& key) {
        std::lock_guard<std::mutex> lock(queryMutex);
        void* reply = redisCommand(context.get(), "EXISTS %s", to_string(key).c_str());
        if (!reply) {
            printf("coud not receive result form database: %s\n", context->errstr);
            return false;
        }
        redisReply* redisrep = (redisReply*)reply;
        bool result = redisrep->integer;
        freeReplyObject(reply);

        return result;
    }

    void RedisMeasurementStorage::deleteDatabase() {
        std::lock_guard<std::mutex> lock(queryMutex);
        redisCommand(context.get(), "flushdb");
    }

}  // namespace slam3d
