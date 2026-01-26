#pragma once
#include <nlohmann/json.hpp>
#include "CommonConfig.h"
#include "MK5NSwerveModule.h"

namespace RobotConfig
{
struct DriveConfig
{
    static inline const std::string FRONT_LEFT_MODULE = "front_left_module";
    static inline const std::string FRONT_RIGHT_MODULE = "front_right_module";
    static inline const std::string BACK_RIGHT_MODULE = "back_right_module";
    static inline const std::string BACK_LEFT_MODULE = "back_left_module";
    static inline const std::string SWERVE_KINEMATICS = "swerve_kinematics";
    
    HighVoltage::MK5NSwerveModule::Mk5NConfig front_left_module;
    HighVoltage::MK5NSwerveModule::Mk5NConfig front_right_module;
    HighVoltage::MK5NSwerveModule::Mk5NConfig back_left_module;
    HighVoltage::MK5NSwerveModule::Mk5NConfig back_right_module;    
    HighVoltage::SwerveKinematicsConfig swerve_kinematics;


    friend void from_json(const nlohmann::json &j, DriveConfig &drive);
};

struct RobotConfig
{
    static inline const std::string DRIVE = "drive";

    DriveConfig drive_config;
    friend void from_json(const nlohmann::json &j, RobotConfig &robot);
};
}