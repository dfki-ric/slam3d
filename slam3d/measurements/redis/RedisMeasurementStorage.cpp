#include <hiredis/hiredis.h>

#include "RedisMeasurementStorage.hpp"

#include <boost/archive/text_iarchive.hpp>


namespace slam3d {

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

        Measurement::Ptr measurement = MeasurementSerialization::deserialize(redisrep->element[1]->str, redisrep->element[0]->str);
        //todo: create from 

        // redisReaderFree(reader);
        freeReplyObject(reply);
        return measurement;
    }

}  // namespace slam3d
