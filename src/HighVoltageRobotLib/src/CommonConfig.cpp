#include "CommonConfig.h"

namespace HighVoltage
{

void from_json(const nlohmann::json &j, PIDConfig &pid)
{
    j.at(PIDConfig::K_P).get_to(pid.k_p);
    j.at(PIDConfig::K_I).get_to(pid.k_i);
    j.at(PIDConfig::K_D).get_to(pid.k_d);
}

void from_json(const nlohmann::json &j, FeedForwardConfig &feed_forward)
{
    j.at(FeedForwardConfig::K_S).get_to(feed_forward.k_s);
    j.at(FeedForwardConfig::K_V).get_to(feed_forward.k_v);
    j.at(FeedForwardConfig::K_A).get_to(feed_forward.k_a);
}

void from_json(const nlohmann::json &j, ArmFeedForwardConfig &feed_forward)
{
    j.at(ArmFeedForwardConfig::K_S).get_to(feed_forward.k_s);
    j.at(ArmFeedForwardConfig::K_V).get_to(feed_forward.k_v);
    j.at(ArmFeedForwardConfig::K_A).get_to(feed_forward.k_a);
    j.at(ArmFeedForwardConfig::K_G).get_to(feed_forward.k_g);
}

void from_json(const nlohmann::json &j, TrapazoidalProfileConfig &tp)
{
    j.at(TrapazoidalProfileConfig::MAX_VELOCITY).get_to(tp.max_velocity);
    j.at(TrapazoidalProfileConfig::MAX_ACCELERATION).get_to(tp.max_acceleration);
}

void from_json(const nlohmann::json &j, RobotBaseSize &robot)
{
    j.at(RobotBaseSize::WHEEL_BASE).get_to(robot.wheel_base);
    j.at(RobotBaseSize::TRACK_WIDTH).get_to(robot.track_width);
}

void from_json(const nlohmann::json &j, HighVoltage::MK4SwerveModuleConfig &module)
{
    j.at(HighVoltage::MK4SwerveModuleConfig::DRIVE_ID).get_to(module.drive_id);
    j.at(HighVoltage::MK4SwerveModuleConfig::ROTATE_ID).get_to(module.rotate_id);
    j.at(HighVoltage::MK4SwerveModuleConfig::ENCODER_ID).get_to(module.encoder_id);
    j.at(HighVoltage::MK4SwerveModuleConfig::ENCODER_OFFSET).get_to(module.encoder_offset);
    j.at(HighVoltage::MK4SwerveModuleConfig::DRIVE_PID).get_to(module.drive_pid);
    j.at(HighVoltage::MK4SwerveModuleConfig::ANGLE_PID).get_to(module.angle_pid);
    j.at(HighVoltage::MK4SwerveModuleConfig::ANGLE_CONSTRAINTS).get_to(module.angle_constraints);
    j.at(HighVoltage::MK4SwerveModuleConfig::ANGLE_FEEDFORWARD).get_to(module.angle_feedforward);
    j.at(HighVoltage::MK4SwerveModuleConfig::DRIVE_FEEDFORWARD).get_to(module.drive_feedforward);
}

void from_json(const nlohmann::json &j, Position2DConfig &p2d)
{
    j.at(Position2DConfig::X_KEY).get_to(p2d.x);
    j.at(Position2DConfig::Y_KEY).get_to(p2d.y);
}

void from_json(const nlohmann::json& j, SwerveKinematicsConfig& sk)
{
    j.at(SwerveKinematicsConfig::FRONT_LEFT_POSITION_KEY).get_to(sk.front_left_position);
    j.at(SwerveKinematicsConfig::FRONT_RIGHT_POSITION_KEY).get_to(sk.front_right_position);
    j.at(SwerveKinematicsConfig::BACK_LEFT_POSITION_KEY).get_to(sk.back_left_position);
    j.at(SwerveKinematicsConfig::BACK_RIGHT_POSITION_KEY).get_to(sk.back_right_position);
}



} // namespace HighVoltage