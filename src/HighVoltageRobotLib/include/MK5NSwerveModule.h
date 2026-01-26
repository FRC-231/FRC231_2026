#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <memory>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>

#include "SwerveModule.h"
#include "CommonConfig.h"
#include "Utility.h"


namespace HighVoltage
{

class MK5NSwerveModule : public SwerveModule
{
public:

    struct Mk5NConfig
    {
        uint8_t drive_id; //CAN ID for the drive motor
        uint8_t rotate_id; //CAN ID for the rotation motor
        uint8_t encoder_id; //CAN ID for the encoder
        units::angle::degree_t encoder_offset; //The magnets in the encoders are not alligned with the wheels. This offset corrects for that.

        FeedForwardConfig rotate_ff; //The feedforward for velocity of the rotation motor
        PIDConfig rotate_velocity_pid; //Helps the feed forward control velocity of the rotation motor
        PIDConfig rotate_angle_pid; //Calculates the velocity setpoint for the feedforward + velocity pid to reach the desired angle

        friend void from_json(const nlohmann::json& j, Mk5NConfig& config); //Friend function to convert from json
    };

    MK5NSwerveModule() = delete;
    MK5NSwerveModule(std::string name,
        Mk5NConfig config);

    void update(std::chrono::duration<double> seconds_passed) override;
    frc::SwerveModulePosition getModulePosition() override;
    units::angle::degree_t getRawAngle() override;
    units::angle::degree_t getAngle() override;
    units::length::meter_t getEncoderPosition() override;

private:

    units::angle::degree_t target_angle{0};
    units::velocity::meters_per_second_t target_speed{0}; //TODO: Make degrees per second
    std::string module_name;

    Mk5NConfig config;

    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> drive_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> rotate_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
    std::unique_ptr<frc::SimpleMotorFeedforward<units::angle::degree_t>> rotate_feedforward;
    std::unique_ptr<frc::PIDController> rotate_velocity_pid;
    std::unique_ptr<frc::PIDController> rotate_angle_pid;



};
} // namespace HighVoltage
