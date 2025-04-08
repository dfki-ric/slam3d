#pragma once

#include <string>
#include <neo4j-client.h>


class Neo4jValue {
 public:
    Neo4jValue(const neo4j_value_t value):value(value){}

    Neo4jValue(const neo4j_result_t *result, const size_t& index = 0){
        value = neo4j_result_field(result, 0);
    }

    int as_integer() {
        return neo4j_int_value(value);
    }

    bool as_bool() {
        return neo4j_bool_value(value);
    }

    std::string as_string() {
        std::string result;
        result.resize(neo4j_string_length(value)+1); // includes '\0' in copy
        neo4j_string_value(value, const_cast<char*>(result.data()), result.size());
        result.resize(result.size()-1);
        return result;
    }

    std::string to_string(const size_t &buffersize = 1024) {
        std::string buf;
        buf.resize(buffersize);
        size_t size = neo4j_ntostring(value, buf.data(), buf.size());
        while (size == buffersize) {
            buf.resize(buf.size() + buffersize);
            size = neo4j_ntostring(value, buf.data(), buf.size());
        }
        buf.resize(size);
        return buf;
    }

    Neo4jValue as_node_properties() {
        return Neo4jValue(neo4j_node_properties(value));
    }

    std::map<std::string, std::string> as_node_properties_map(){
        std::map<std::string, std::string> result;
        neo4j_value_t map = neo4j_node_properties(value);
        size_t size = neo4j_map_size(map);
        for (size_t i = 0; i< size; ++i) {
            const neo4j_map_entry_t *entry = neo4j_map_getentry(map,i);
            Neo4jValue key(entry->value);
            Neo4jValue value(entry->value);
            result[key.as_string()] = value.to_string();
        }
        return result;
    }

    Neo4jValue as_relationship_properties() {
        return Neo4jValue(neo4j_relationship_properties(value));
    }

    std::map<std::string, std::string> as_relationship_properties_map(){
        std::map<std::string, std::string> result;
        neo4j_value_t map = neo4j_relationship_properties(value);
        size_t size = neo4j_map_size(map);
        for (size_t i = 0; i< size; ++i) {
            const neo4j_map_entry_t *entry = neo4j_map_getentry(map,i);
            Neo4jValue key(entry->value);
            Neo4jValue value(entry->value);
            result[key.as_string()] = value.to_string();
        }
        return result;
    }

    Neo4jValue operator[](const std::string& key) {
        return Neo4jValue(neo4j_map_kget(value, neo4j_ustring(key.data(), key.size())));
    }

    void print() {
        neo4j_fprint(value, stdout);
        printf("\n");
    }

    void printType() {
        printf("%s\n",neo4j_typestr(value._type));
    }

 private:
    neo4j_value_t value;
};