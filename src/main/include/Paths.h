#pragma once
#include <filesystem>
#include <frc/Filesystem.h>
#include <wpi/fs.h>

namespace Paths
{
    inline const std::filesystem::path BASE_ROBOT_CONFIG = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"config"/"BaseRobot.json";
}