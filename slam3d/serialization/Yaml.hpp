# pragma once

#include <map>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>


/**
 * @brief Yaml<MYCONFIG>& config = Yaml<MYCONFIG>::getInstance(); you need to define a 
 * yaml-cpp compatible parser for your self-defiensd MYCONFIG struct
 * namespace YAML {
 *  template<> struct convert<Config> {
 *      static bool decode(const Node& node, Config& config) {
 *      // you can use the checkAndSet template function defined below to load yaml values 
 *      checkAndSet(&MYCONFIG.string, node["string"]);
 * 
 * @tparam CONFIG a self defined struct
 */
template <class CONFIG> class Yaml {
 public:
    static Yaml& getInstance() {
        static Yaml instance;
        return instance;
    }

    bool loadConfig(const std::string &configfile) {
        config = YAML::LoadFile(configfile);
        loadedFile = configfile;

        // parse based on yaml-cpp config parser defiend for the types
        conf = config.as<CONFIG>();

        return true;
    }

    bool saveConfig(const std::string& file = "") {
        std::string fileToSave = loadedFile;
        if (file != "") {
            fileToSave = file;
        }
        // convert struct to yaml
        config = conf;
        // write
        std::ofstream fout(fileToSave);
        if (fout.is_open()) {
            YAML::Emitter emitter;
            // emitter.SetSeqFormat(YAML::Flow);
            emitter << config;
            fout << emitter.c_str();
            fout.close();
            return true;
        }
        return false;
    }


 private:
    // private constructor to force usage of getInstance()
    Yaml() {}


 public:
    CONFIG& get() {
        return conf;
    }

 private:
    std::string loadedFile;
    CONFIG conf;

    YAML::Node config;
};

/**
 * @brief template to parse a configuration entry Configuration::checkAndSet(config.device, node["device"]);
 * 
 * @tparam TARGET some yaml-cpp parser supported cpp variable
 * @param dst the cpp variable to write yaml value to
 * @param src the node with the config value
 * @return true 
 * @return false 
 */
template <class TARGET> bool checkAndSet(TARGET *dst, const YAML::Node src) {
    if (src) {
        *dst = src.as<TARGET>();
        return true;
    }
    return false;
}

template <class TARGET> bool checkAndSet(TARGET *dst, const YAML::Node src, const TARGET &defaultValue) {
    if (!checkAndSet(dst, src)) {
        *dst = defaultValue;
    }
    return true;
}
