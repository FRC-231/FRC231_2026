#include "RobotContext.h"
#include <iostream>
#include <fstream>

RobotContext::RobotContext()
{

}

bool RobotContext::loadConfig(const std::filesystem::path& path)
{
    std::cout << "loading config file " << path << std::endl;
    std::ifstream file(path);
    if(!file.is_open()) 
    {
        std::cout << "error opening file" << std::endl;
        return false;
    }
    nlohmann::json j;
    file >> j;
    file.close();
    try
    {
        RobotConfig::RobotConfig temp_config;
        j.get_to(temp_config);
        config = temp_config;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    return true;
    
}

const std::optional<RobotConfig::RobotConfig>& RobotContext::getConfig() const
{
    return config;
}

frc::PS5Controller& RobotContext::getPilotController()
{
    return pilot_controller;
}