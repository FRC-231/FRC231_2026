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
    static inline const std::string PIVOT_WIGGLE_UP_ANGLE = "pivot_wiggle_up_angle";
    static inline const std::string PIVOT_WIGGLE_DOWN_ANGLE = "pivot_wiggle_down_angle";
    static inline const std::string PIVOT_WIGGLE_PERIOD = "pivot_wiggle_period";
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
    double pivot_wiggle_up_angle;
    double pivot_wiggle_down_angle;
    double pivot_wiggle_period;
    double pivot_encoder_offset;

    friend void from_json(const nlohmann::json &j, IntakeConfig &intake);
};

struct ShooterConfig
{
    static inline const std::string HOPPER_MOTOR_ID = "hopper_motor_id";
    static inline const std::string FEED_MOTOR_ID = "feed_motor_id";
    static inline const std::string LEFT_SHOOTER_MOTOR_ID = "left_shooter_motor_id";
    static inline const std::string RIGHT_SHOOTER_MOTOR_ID = "right_shooter_motor_id";
    static inline const std::string SERVO_CONTROLLER_ID = "servo_controller_id";
    static inline const std::string LEFT_SERVO_ID = "left_servo_id";
    static inline const std::string RIGHT_SERVO_ID = "right_servo_id";
    static inline const std::string SHOOTER_PID = "shooter_pid";
    static inline const std::string HOPPER_VOLTAGE = "hopper_voltage";
    static inline const std::string HOPPER_WIGGLE_LOW_VOLTAGE = "hopper_wiggle_low_voltage";
    static inline const std::string HOPPER_WIGGLE_PERIOD = "hopper_wiggle_period";
    static inline const std::string FEED_VOLTAGE = "feed_voltage";
    static inline const std::string SHOOTER_FF = "shooter_ff";
    static inline const std::string SHOOTER_GEAR_RATIO = "shooter_gear_ratio";
    static inline const std::string SHOOTER_MAX_VOLTAGE = "shooter_max_voltage";
    static inline const std::string SHOOTER_MIN_VOLTAGE = "shooter_min_voltage";
    static inline const std::string BACKSPIN_AMOUNT = "backspin_amount";
    static inline const std::string DISTANCE_TO_SHOOTER_VELOCITY = "distance_to_shooter_velocity";
    static inline const std::string DISTANCE_TO_BACKSPIN = "distance_to_backspin";
    static inline const std::string DISTANCE_TO_PULSEWIDTH = "distance_to_pulsewidth";
    static inline const std::string AT_SPEED_TOLERANCE = "at_speed_tolerance";
    static inline const std::string IDLE_MODE_VELOCITY = "idle_mode_velocity";
    static inline const std::string SNOW_BLOW_TARGET_RPM = "snow_blow_target_rpm";
    static inline const std::string SNOW_BLOW_PULSE_WIDTH = "snow_blow_pulse_width";


    uint8_t hopper_motor_id;
    uint8_t feed_motor_id;
    uint8_t left_shooter_motor_id;
    uint8_t right_shooter_motor_id;
    uint8_t servo_controller_id;
    uint8_t left_servo_id;
    uint8_t right_servo_id;
    HighVoltage::PIDConfig shooter_pid;
    HighVoltage::FeedForwardConfig shooter_ff;
    double hopper_voltage;
    double hopper_wiggle_low_voltage;
    double hopper_wiggle_period;
    double feed_voltage;
    double shooter_gear_ratio;
    double shooter_max_voltage;
    double shooter_min_voltage;
    double backspin_amount;
    double at_speed_tolerance;
    double idle_mode_velocity;
    double snow_blow_target_rpm;
    int snow_blow_pulse_width;
    std::map<double, double> distance_to_shooter_velocity;
    std::map<double, double> distance_to_backspin;
    std::map<double, double> distance_to_pulsewidth;

    friend void from_json(const nlohmann::json &j, ShooterConfig &shooter);
};

struct RobotConfig
{
    static inline const std::string DRIVE = "drive";
    static inline const std::string INTAKE = "intake";
    static inline const std::string SHOOTER = "shooter";
    static inline const std::string COMPETITION_MODE = "competition_mode";
    static inline const std::string NT_THROTTLE_INTERVAL = "nt_throttle_interval";

    DriveConfig drive_config;
    IntakeConfig intake_config;
    ShooterConfig shooter_config;
    bool competition_mode = false;
    int nt_throttle_interval = 5; // loops to skip when throttled (5 = ~10Hz at 50Hz loop)
    friend void from_json(const nlohmann::json &j, RobotConfig &robot);
};
}