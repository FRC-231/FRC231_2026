#pragma once
#include <nlohmann/json.hpp>
#include "CommonConfig.h"

namespace RobotConfig
{
struct DriveConfig
{
    static inline const std::string FRONT_LEFT_MODULE = "front_left_module";
    static inline const std::string FRONT_RIGHT_MODULE = "front_right_module";
    static inline const std::string BACK_RIGHT_MODULE = "back_right_module";
    static inline const std::string BACK_LEFT_MODULE = "back_left_module";
    static inline const std::string SWERVE_KINEMATICS = "swerve_kinematics";
    static inline const std::string ROTATE_FEED_FORWARD = "rotate_feed_forward";
    static inline const std::string DRIVE_FEED_FORWARD = "drive_feed_forward";
    struct SwerveModuleInfo
    {
        static inline const std::string ROTATE_ID = "rotate";
        static inline const std::string DRIVE_ID = "drive";
        static inline const std::string ENCODER_ID = "encoder";
        static inline const std::string ENCODER_OFFSET = "encoder_offset";

        uint8_t rotate_id;
        uint8_t drive_id;
        uint8_t encoder_id;
        double encoder_offset;

        friend void from_json(const nlohmann::json &j, SwerveModuleInfo &module);
    };

    HighVoltage::SwerveKinematicsConfig swerve_kinematics;
    HighVoltage::FeedForwardConfig rotate_feed_forward;
    HighVoltage::FeedForwardConfig drive_feed_forward;
    SwerveModuleInfo front_left_module;
    SwerveModuleInfo front_right_module;
    SwerveModuleInfo back_right_module;
    SwerveModuleInfo back_left_module;

    friend void from_json(const nlohmann::json &j, DriveConfig &drive);
};

struct RobotConfig
{
    static inline const std::string DRIVE = "drive";

    DriveConfig drive_config;
    friend void from_json(const nlohmann::json &j, RobotConfig &robot);
};
}