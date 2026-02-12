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
    static inline const std::string ANGLE_CORRECTION_PID = "angle_correction_pid";
    static inline const std::string DRIVE_TO_PROPORTIONAL = "drive_to_proportional";

    
    HighVoltage::MK5NSwerveModule::Mk5NConfig front_left_module;
    HighVoltage::MK5NSwerveModule::Mk5NConfig front_right_module;
    HighVoltage::MK5NSwerveModule::Mk5NConfig back_left_module;
    HighVoltage::MK5NSwerveModule::Mk5NConfig back_right_module;    
    HighVoltage::SwerveKinematicsConfig swerve_kinematics;
    HighVoltage::PIDConfig angle_correction_pid;
    double drive_to_proportional;


    friend void from_json(const nlohmann::json &j, DriveConfig &drive);
};
struct IntakeConfig
{
    static inline const std::string PIVOT_MOTOR_ID = "pivot_motor_id";
    static inline const std::string FEED_MOTOR_ID = "feed_motor_id";
    static inline const std::string PIVOT_ENCODER_ID = "pivot_encoder_id";
    static inline const std::string FEED_INTAKE_VOLTAGE = "feed_intake_voltage";
    static inline const std::string FEED_OUTTAKE_VOLTAGE = "feed_outtake_voltage";
    static inline const std::string PIVOT_MAX_VOLTAGE = "pivot_max_voltage";
    static inline const std::string PIVOT_MIN_VOLTAGE = "pivot_min_voltage";
    static inline const std::string PIVOT_MAX_ANGLE = "pivot_max_angle";
    static inline const std::string PIVOT_MIN_ANGLE = "pivot_min_angle";
    static inline const std::string PIVOT_PID = "pivot_pid";
    static inline const std::string PIVOT_UP_ANGLE = "pivot_up_angle";
    static inline const std::string PIVOT_DOWN_ANGLE = "pivot_down_angle";
    static inline const std::string PIVOT_ENCODER_OFFSET = "pivot_encoder_offset";



    uint8_t pivot_motor_id;
    uint8_t feed_motor_id;
    uint8_t pivot_encoder_id;
    double feed_intake_voltage;
    double feed_outtake_voltage;
    double pivot_max_voltage;
    double pivot_min_voltage;
    double pivot_max_angle;
    double pivot_min_angle;
    HighVoltage::PIDConfig pivot_pid;
    double pivot_up_angle;
    double pivot_down_angle;
    double pivot_encoder_offset;

    friend void from_json(const nlohmann::json &j, IntakeConfig &drive);
};

struct RobotConfig
{
    static inline const std::string DRIVE = "drive";
    static inline const std::string INTAKE = "intake";

    DriveConfig drive_config;
    IntakeConfig intake_config;
    friend void from_json(const nlohmann::json &j, RobotConfig &robot);
};
}