#pragma once
#include <filesystem>
#include <optional>
#include "RobotConfig.h"
#include <frc/PS5Controller.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <Vision.h>

class RobotContext
{
public:
    RobotContext();
    bool loadConfig(const std::filesystem::path& path);
    const std::optional<RobotConfig::RobotConfig>& getConfig() const;
    frc::PS5Controller& getPilotController();
    ctre::phoenix6::hardware::Pigeon2& getPigeon();
    void setStartAngle(units::angle::degree_t angle);
    units::angle::degree_t getStartAngle();
    Vision& getVision();

private:
    std::optional<RobotConfig::RobotConfig> config;
    frc::PS5Controller pilot_controller{0};
    ctre::phoenix6::hardware::Pigeon2 pigeon{30};
    units::angle::degree_t start_angle{0.0};
    Vision vision;
};
