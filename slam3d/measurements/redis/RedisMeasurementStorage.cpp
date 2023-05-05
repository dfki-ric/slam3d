#include <hiredis/hiredis.h>
#include <boost/serialization/shared_ptr.hpp>

#include "RedisMeasurementStorage.hpp"


namespace slam3d {

    void RedisMeasurementStorage::add(Measurement::Ptr measurement) {
        std::stringstream ss;
        boost::archive::text_oarchive oa(ss);
        oa << measurement;
        store(to_string(measurement->getUniqueId()), measurement->getTypeName(), ss.str());
    }

    RedisMeasurementStorage::RedisMeasurementStorage(const char *ip, int port):MeasurementStorage() {
        context = std::shared_ptr<redisContext>(redisConnect(ip, port));

        if (context.get() == nullptr || context->err) {
            if (context.get()) {
                printf("Error: %s\n", context->errstr);
                // handle error
            } else {
                printf("Can't allocate redis context\n");
            }
        }
        redisCommand(context.get(), "flushdb");
    }

    void RedisMeasurementStorage::store(const std::string& key, const std::string &type, const std::string& serializedData) {
        // printf("HSET %s type %s, data %s\n", key.c_str(), type.c_str(), "data");
        void* reply = redisCommand(context.get(), "HSET %s type %s data %s", key.c_str(), type.c_str(), serializedData.c_str());
        if (!reply) {
            printf("coud not store measurement: %s\n", context->errstr);
            // TODO: recreate context
            return;
        }
        // redisReply* redisrep = (redisReply*)reply;
        // printf("reply %i %lu\n", redisrep->type,  redisrep->elements );
        freeReplyObject(reply);
    }

    Measurement::Ptr RedisMeasurementStorage::get(const std::string& key) {
        void* reply = redisCommand(context.get(), "HMGET %s type data", key.c_str());
        if (!reply) {
            printf("coud not load measurement: %s\n", context->errstr);
            // TODO: recreate context
            freeReplyObject(reply);
            return Measurement::Ptr();
        }
        redisReply* redisrep = (redisReply*)reply;
        // printf("reply %i %lu\n", redisrep->type,  redisrep->elements );

        // printf("got %s\n", redisrep->element[0]->str);

        std::stringstream data(redisrep->element[1]->str);
        boost::archive::text_iarchive ia(data);
        Measurement::Ptr measurement;
        ia >> measurement;

        //todo: create from 

        // redisReaderFree(reader);
        freeReplyObject(reply);
        return measurement;
    }

    Measurement::Ptr RedisMeasurementStorage::get(const boost::uuids::uuid& key) {
        return get(to_string(key));
    }

}  // namespace slam3d
