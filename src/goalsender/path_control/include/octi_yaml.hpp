#ifndef OCTI_YAML__HPP
#define OCTI_YAML__HPP
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>

class octi_yaml
{
private:
    const char* filePath;
    YAML::Node yaml;
public:
    octi_yaml(const char* filePath_);
    ~octi_yaml();
    double getDouble(const char* item);
    unsigned int getUint(const char* item);
    std::string getString(const char* item);
    bool IsDefine(const char* item);
};

#endif