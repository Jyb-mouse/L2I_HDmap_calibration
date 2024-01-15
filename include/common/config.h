#pragma once 

#include <string>
#include <list>
#include <map> 
#include <sstream>
#include <iostream>

class Config{
 public:
        explicit Config(std::string config_name);
        Config();

        ~Config() = default;
        bool LoadFile(std::string config_name);

    template <typename T>
    T getvalue(std::string key){
        std::string str = data_map_[key];
        if (str == "")
        {
            std::cerr << "WARNING: '" << key <<"' was not defined in " << config_name_ << "! Value is undefined!" << std::endl;
        }
        std::stringstream ss;
        ss << str;
        T value;
        ss >> value;
        return value;        
    }

    template <typename T>
    T getvalue(std::string key, T defaultValue){
        std::string str = data_map_[key];
        if (str == "")
        {
            return defaultValue;
        }
        return getvalue<T>(key);
    } 

     
    template < typename T>
    T getvalueidx(std::string key,int idx, T defaultValue) {
        std::stringstream ss;
        ss << idx;
            std::string query=key+std::string("[")+ss.str()+std::string("]");
        return getvalue<T>(query,defaultValue);
    }
    
    template < typename T>
    T getvalueidx(std::string key,int idx) {
        std::stringstream ss;
        ss << idx;
        std::string query=key+std::string("[")+ss.str()+std::string("]");
        return getvalue<T>(query);
    }

 private:
        std::string config_name_;
        std::map<std::string, std::string> data_map_; 
};