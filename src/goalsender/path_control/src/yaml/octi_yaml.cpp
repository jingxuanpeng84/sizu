#include "path_control/include/octi_yaml.hpp"
octi_yaml::octi_yaml(const char* filePath_)
{
    this->filePath = filePath_;
    this->yaml = YAML::LoadFile(filePath_);
}

octi_yaml::~octi_yaml()
{
}

unsigned int octi_yaml::getUint(const char* item)
{
    if(!(this->yaml)[item].IsDefined())
    {
        return 0;
    }
    return (this->yaml)[item].as<u_int32_t>();
}

std::string octi_yaml::getString(const char* item)
{
    std::string str = "nan";
    if(!(this->yaml)[item].IsDefined())
    {
        return str;
    }
    return (this->yaml)[item].as<std::string>();
}

double octi_yaml::getDouble(const char* item)
{
    if(!(this->yaml)[item].IsDefined())
    {
        return 0;
    }
    return (this->yaml)[item].as<double>();
}

bool octi_yaml::IsDefine(const char* item)
{
    return (this->yaml)[item].IsDefined();
}