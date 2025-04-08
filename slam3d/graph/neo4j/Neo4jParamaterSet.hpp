#pragma once

#include <neo4j-client.h>
#include <vector>
#include <list>
#include <map>

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
    std::list<std::string> names; // <- using a list because memory location must not be changed, neo4j_string() requies it

    // std::map<std::string, neo4j_value_t> set_values;
    // storage to save the above map content
    std::map<std::string, std::vector<neo4j_map_entry_t> > setmaps_storage;
    std::map<std::string, std::vector<std::string> > setmaps_storage_paramnames;

    

};
