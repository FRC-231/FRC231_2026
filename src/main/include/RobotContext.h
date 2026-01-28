#pragma once
#include <filesystem>
#include <optional>
#include "RobotConfig.h"
#include <frc/PS5Controller.h>

class RobotContext
{
public:
    RobotContext();
    bool loadConfig(const std::filesystem::path& path);
    const std::optional<RobotConfig::RobotConfig>& getConfig() const;
    frc::PS5Controller& getPilotController();
private:
    std::optional<RobotConfig::RobotConfig> config;
    frc::PS5Controller pilot_controller{0};
};
