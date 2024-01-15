#include "common/config.h"
#include <fstream>

Config::Config(std::string config_name) : config_name_(config_name) {
    if(!LoadFile(config_name))
		std::cerr << "Configfile '" << config_name << "' not found!" << std::endl;
}
Config::Config(){}

bool Config::LoadFile(std::string config_name) {
    std::fstream f;
    f.open(config_name.c_str(), std::fstream::in);
    if ((!f.is_open()))
    {
        return false;
    }
    std::string line;
    int lnr = 1;
    while (std::getline(f, line))
    {
        lnr++;
        if (!line.length()) continue;
        if (line[0] == '#') continue;
        if (line[0] == ';') continue;
        int posTrenner = line.find('=');
        if (posTrenner == -1)
        {
            posTrenner = line.find(' ');
        }
        if (posTrenner==-1) {
			std::cerr << "WARNING: Statement '" << line << "' in file "<< config_name << ":"<<lnr<<" is invalid and therefor will be ignored" << std::endl;
			continue;
		}
		std::string key=line.substr(0,posTrenner);
		std::string value=line.substr(posTrenner+1);
		if (data_map_[key]!="") {
			std::cerr << "WARNING: Statement '" << line << "' in file "<< config_name << ":"<<lnr<<" redefines a value!" << std::endl;
		}
		data_map_[key]=value;
    }
    f.close();
    return true;
}
