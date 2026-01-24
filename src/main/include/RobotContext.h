#pragma once
#include <filesystem>
#include <optional>
#include "RobotConfig.h"

class RobotContext
{
public:
    RobotContext();
    bool loadConfig(const std::filesystem::path& path);
    const std::optional<RobotConfig::RobotConfig>& getConfig() const;
private:
    std::optional<RobotConfig::RobotConfig> config;
};
