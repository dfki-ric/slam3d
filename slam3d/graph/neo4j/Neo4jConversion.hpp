#pragma once

#include <string>
#include <list>
// dont define the U macro in the http client, it conflicts with the eigen U macro use _XPLATSTR() instead
#define _TURN_OFF_PLATFORM_STRING
#include <cpprest/json.h>
#include <slam3d/core/Types.hpp>

#include <neo4j-client.h>

namespace slam3d {

class MeasurementStorage;


class ParamaterSet {
 public:
    ParamaterSet() {
        sets = neo4j_map(nullptr,0);
    }

    void addParameterSet(const std::string& setname) {
        names.push_back(setname); //needs to stay in class for neo4j_string()
        sets_storage.push_back(neo4j_map_kentry(neo4j_string(names.back().c_str()), neo4j_map(nullptr,0)));
    }

    void addParameterToSet(const std::string& setname, const std::string& paramname, const std::string& value) {
        names.push_back(paramname);
        const char* pname = names.back().c_str();
        names.push_back(value);
        setmaps_storage[setname].push_back(neo4j_map_kentry(neo4j_string(pname), neo4j_string(names.back().c_str())));
    }

    void addParameterToSet(const std::string& setname, const std::string& paramname, const size_t& value) {
        names.push_back(paramname);
        setmaps_storage[setname].push_back(neo4j_map_kentry(neo4j_string(names.back().c_str()), neo4j_int(value)));
    }

    void addBoolParameterToSet(const std::string& setname, const std::string& paramname, const bool& value) {
        names.push_back(paramname);
        setmaps_storage[setname].push_back(neo4j_map_kentry(neo4j_string(names.back().c_str()), neo4j_bool(value)));
    }

    void compile() {
        for (const auto& paramaterSet : setmaps_storage){
            // printf("%s:%i %s\n", __PRETTY_FUNCTION__, __LINE__, paramaterSet.first.c_str());
            // set_values[stored.first] = 
            neo4j_value_t setname = neo4j_string(paramaterSet.first.c_str());
            auto entry = std::find_if(sets_storage.begin(), sets_storage.end(), [&setname](const neo4j_map_entry_t& mapentry) {
                return neo4j_eq(setname,mapentry.key);
            });
            if (entry != sets_storage.end()) {
                //need a find_if?
                entry->value = neo4j_map(paramaterSet.second.data(), paramaterSet.second.size());
            }else{
                printf("%s:%i\n", __PRETTY_FUNCTION__, __LINE__);
            }
        }
        
        sets = neo4j_map(sets_storage.data(), sets_storage.size());
    }

    neo4j_value_t get(){
        compile();
        return sets;
    }

    void print() {
        compile();
        neo4j_fprint(sets, stdout);
        printf("\n");
    }



    neo4j_value_t* getParameterSet(const std::string& setname);

 private:
    
    neo4j_value_t sets;
    // storage to construct the above value
    std::vector<neo4j_map_entry_t> sets_storage;
    std::list<std::string> names; // <- using a list because memory location cannot be changed, neo4j_string() requies it to stay

    // std::map<std::string, neo4j_value_t> set_values;
    // storage to save the above map content
    std::map<std::string, std::vector<neo4j_map_entry_t> > setmaps_storage;
    std::map<std::string, std::vector<std::string> > setmaps_storage_paramnames;

};


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

    Neo4jValue as_node_properties() {
        return Neo4jValue(neo4j_node_properties(value));
    }

    Neo4jValue as_relationship_properties() {
        return Neo4jValue(neo4j_relationship_properties(value));
    }

    Neo4jValue operator[](const std::string& key) {
        return Neo4jValue(neo4j_map_kget(value, neo4j_ustring(key.data(), key.size())));
    }

    void print() {
        neo4j_fprint(value, stdout);
    }

    void printType() {
        printf("%s\n",neo4j_typestr(value._type));
    }

 private:
    neo4j_value_t value;
};



class Neo4jConversion {
 public:
    static std::string eigenMatrixToString(const Eigen::MatrixXd& mat);
    static Eigen::MatrixXd eigenMatrixFromString(const std::string & string);

    static void constraintToJson(slam3d::Constraint::Ptr constraint, web::json::value* json);
    static slam3d::Constraint::Ptr jsonToConstraint(web::json::value& json);

    static slam3d::EdgeObject edgeObjectFromJson(web::json::value& json);
    static slam3d::VertexObject vertexObjectFromJson(web::json::value& json);

    // bolt, libneo4j conversions

    static slam3d::VertexObjectList vertexObjectList(neo4j_result_stream_t *results);
    static slam3d::VertexObject vertexObject(const neo4j_result_t *result);

    static slam3d::EdgeObjectList edgeObjectList(neo4j_result_stream_t *results);
    static slam3d::EdgeObject edgeObject(const neo4j_result_t *result);

    static slam3d::Constraint::Ptr constraint(const neo4j_result_t *result);

    static void constraintToParameters(slam3d::Constraint::Ptr constraint, const std::string& setname, ParamaterSet* set);

    

};

}  // namespace slam3d

