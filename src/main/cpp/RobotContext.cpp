#include "RobotContext.h"
#include <iostream>
#include <fstream>

RobotContext::RobotContext()
{
    // Allow live-toggling competition mode from the dashboard
    nt_competition_mode.setOnUpdate([this](const bool& value)
    {
        competition_mode = value;
    });
    // Allow live-adjusting the NT throttle interval from the dashboard
    nt_throttle_interval_value.setOnUpdate([this](const double& value)
    {
        nt_throttle_interval = static_cast<int>(value);
    });
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
    std::cout << "config loaded" << std::endl;

    // Initialize competition_mode and throttle interval from config and publish to NT
    competition_mode = config->competition_mode;
    nt_competition_mode.setValue(competition_mode);
    nt_throttle_interval = config->nt_throttle_interval;
    nt_throttle_interval_value.setValue(static_cast<double>(nt_throttle_interval));

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

frc::PS5Controller& RobotContext::getCopilotController()
{
    return copilot_controller;
}

frc::XboxController& RobotContext::getPilotXboxController()
{
    return pilot_as_xbox_controller;
}

void RobotContext::setStartAngle(units::angle::degree_t angle)
{
    start_angle = angle;
}

units::angle::degree_t RobotContext::getStartAngle()
{
    return start_angle;
}

ctre::phoenix6::hardware::Pigeon2& RobotContext::getPigeon()
{
    return pigeon;
}

Vision& RobotContext::getVision()
{
    return vision;
}
