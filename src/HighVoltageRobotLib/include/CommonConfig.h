#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <ctre/phoenix6/TalonFX.hpp>

namespace HighVoltage
{

struct PIDConfig
{
    static inline const std::string K_P = "k_p";
    static inline const std::string K_I = "k_i";
    static inline const std::string K_D = "k_d";

    double k_p;
    double k_i;
    double k_d;

    friend void from_json(const nlohmann::json &j, PIDConfig &pid);
};

struct FeedForwardConfig
{
    static inline const std::string K_S = "k_s";
    static inline const std::string K_V = "k_v";
    static inline const std::string K_A = "k_a";

    double k_s;
    double k_v;
    double k_a;

    friend void from_json(const nlohmann::json &j, FeedForwardConfig &feed_forward);
};

struct ArmFeedForwardConfig
{
    static inline const std::string K_S = "k_s";
    static inline const std::string K_V = "k_v";
    static inline const std::string K_A = "k_a";
    static inline const std::string K_G = "k_g";

    double k_s;
    double k_v;
    double k_a;
    double k_g;

    friend void from_json(const nlohmann::json &j, ArmFeedForwardConfig &feed_forward);
};

struct RobotBaseSize
{
    static inline const std::string WHEEL_BASE = "wheel_base";
    static inline const std::string TRACK_WIDTH = "track_width";

    double wheel_base; // distance between the wheels
    double track_width; // distance between the front and back wheels
    friend void from_json(const nlohmann::json &j, RobotBaseSize &robot);
};

struct TrapazoidalProfileConfig
{

    static inline const std::string MAX_VELOCITY = "max_velocity";
    static inline const std::string MAX_ACCELERATION = "max_acceleration";

    double max_velocity{0};
    double max_acceleration{0};
    friend void from_json(const nlohmann::json& j, TrapazoidalProfileConfig& tp);
};

struct Position2DConfig
{
    static inline const std::string X_KEY = "x";
    static inline const std::string Y_KEY = "y";

    double x{0};
    double y{0};

    friend void from_json(const nlohmann::json& j, Position2DConfig& p2d);
};

struct SwerveKinematicsConfig
{
    static inline const std::string FRONT_LEFT_POSITION_KEY = "front_left";
    static inline const std::string FRONT_RIGHT_POSITION_KEY = "front_right";
    static inline const std::string BACK_LEFT_POSITION_KEY = "back_left";
    static inline const std::string BACK_RIGHT_POSITION_KEY = "back_right";

    Position2DConfig front_left_position;
    Position2DConfig front_right_position;
    Position2DConfig back_left_position;
    Position2DConfig back_right_position;

    friend void from_json(const nlohmann::json& j, SwerveKinematicsConfig& sk);
};


struct MK4SwerveModuleConfig
{
    static inline const std::string DRIVE_ID = "drive_id";
    static inline const std::string ROTATE_ID = "rotate_id";
    static inline const std::string ENCODER_ID = "encoder_id";
    static inline const std::string ENCODER_OFFSET = "encoder_offset";
    static inline const std::string DRIVE_PID = "drive_pid";
    static inline const std::string ANGLE_PID = "angle_pid";
    static inline const std::string ANGLE_CONSTRAINTS = "angle_constraints";
    static inline const std::string ANGLE_FEEDFORWARD = "angle_feedforward";
    static inline const std::string DRIVE_FEEDFORWARD = "drive_feedforward";

    uint8_t drive_id;
    uint8_t rotate_id;
    uint8_t encoder_id;
    double encoder_offset;
    PIDConfig drive_pid;
    PIDConfig angle_pid;
    TrapazoidalProfileConfig angle_constraints;
    FeedForwardConfig angle_feedforward;
    FeedForwardConfig drive_feedforward;

    friend void from_json(const nlohmann::json &j, MK4SwerveModuleConfig &module);
};

template <uint8_t slot = 0>
void applyMotionConfigToTalonFX(const PIDConfig& pid_config, const FeedForwardConfig& ff_config, ctre::phoenix6::hardware::TalonFX& motor)
{
    using SlotType =
        std::conditional_t<slot == 0, ctre::phoenix6::configs::Slot0Configs,
        std::conditional_t<slot == 1, ctre::phoenix6::configs::Slot1Configs,
        std::conditional_t<slot == 2, ctre::phoenix6::configs::Slot2Configs,
        void>>>;

    SlotType slot_config;
    slot_config.kP = pid_config.k_p;
    slot_config.kI = pid_config.k_i;
    slot_config.kD = pid_config.k_d;
    slot_config.kS = ff_config.k_s;
    slot_config.kV = ff_config.k_v;
    slot_config.kA = ff_config.k_a;

    motor.GetConfigurator().Apply(slot_config);
}



} // namespace HighVoltage
