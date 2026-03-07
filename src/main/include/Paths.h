#pragma once
#include <filesystem>
#include <frc/Filesystem.h>
#include <wpi/fs.h>

namespace Paths
{
    inline const std::filesystem::path BASE_ROBOT_CONFIG = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"config"/"BaseRobot.json";
    inline const std::filesystem::path TEST_ROBOT_CONFIG = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"config"/"TestRobot.json";
    inline const std::filesystem::path AUTON_DIRECTORY = std::filesystem::path(frc::filesystem::GetDeployDirectory())/"scripts"/"autons";
    inline const std::filesystem::path TEST_AUTON = AUTON_DIRECTORY/"TestAuton.lua";
    inline const std::filesystem::path DANCE_AUTON = AUTON_DIRECTORY/"Dance.lua";
    inline const std::filesystem::path BUMP_TEST_AUTON = AUTON_DIRECTORY/"BumpTest.lua";
    inline const std::filesystem::path SCORE_TEST_AUTON = AUTON_DIRECTORY/"ScoreTest.lua";
    inline const std::filesystem::path QUICK_LOAD_AUTON = AUTON_DIRECTORY/"QuickLoad.lua";
}