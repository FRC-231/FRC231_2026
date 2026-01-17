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

class Mk4SwerveModule : public SwerveModule
{
public:
    Mk4SwerveModule(std::string name,
        uint8_t drive_id,
        uint8_t rotate_id,
        uint8_t encoder_id,
        units::angle::degree_t encoder_offset,
        std::shared_ptr<frc::PIDController> drive_pid,
        std::shared_ptr<frc::ProfiledPIDController<units::meters>> angle_pid,
        std::shared_ptr<frc::SimpleMotorFeedforward<units::meters>> angle_feedforward,
        std::shared_ptr<frc::SimpleMotorFeedforward<units::meters>> drive_feedforward);
    Mk4SwerveModule(std::string name,
        uint8_t drive_id,
        uint8_t rotate_id,
        uint8_t encoder_id,
        units::angle::degree_t encoder_offset,
        PIDConfig drive_pid_config,
        PIDConfig angle_pid_config,
        TrapazoidalProfileConfig angle_constraints,
        FeedForwardConfig angle_feedforward_config,
        FeedForwardConfig drive_feedforward_config);

    units::angle::degree_t ALL_ENCODER_OFFSET{180.0};

    // Override functions
    units::angle::degree_t getAngle() override;
    units::angle::degree_t getRawAngle() override;
    units::length::meter_t getEncoderPosition() override;
    frc::SwerveModulePosition getModulePosition() override;
    void update(std::chrono::duration<double> seconds_passed) override;
    void resetError();

    void populateNetworkTable();

    units::angle::degree_t TURN_MOTOR_DEADZONE{1.5};
    static constexpr double TURN_TO_METER_RATIO = 0.0521;
   // static constexpr double TURN_TO_METER_RATIO = 0.0456; //2024 Robot

private:
    units::angle::degree_t offset;

    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> drive_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> rotate_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
    std::shared_ptr<frc::PIDController> drive_pid;
    std::shared_ptr<frc::ProfiledPIDController<units::meters>> angle_pid;
    std::shared_ptr<frc::SimpleMotorFeedforward<units::meters>> angle_feedforward;
    std::shared_ptr<frc::SimpleMotorFeedforward<units::meters>> drive_feedforward;

    std::string network_table_name;

    //NetworkTable stuff
    void setupNetworkTable();
    nt::NetworkTableInstance network_table_inst;
    std::shared_ptr<nt::NetworkTable> module_table;
    nt::DoubleTopic drive_speed_topic;
    nt::DoublePublisher drive_speed_publisher;
    nt::DoubleTopic drive_target_topic;
    nt::DoublePublisher drive_target_publisher;
    nt::DoubleTopic raw_angle_topic;
    nt::DoublePublisher raw_angle_publisher;
    nt::DoubleTopic angle_topic;
    nt::DoublePublisher angle_publisher;
    nt::DoubleTopic target_angle_topic;
    nt::DoublePublisher target_angle_publisher;
    
    nt::DoubleTopic rotate_voltage_topic;
    nt::DoublePublisher rotate_voltage_publisher;
    
    nt::DoubleTopic rotate_feedforward_voltage_topic;
    nt::DoublePublisher rotate_feedforward_voltage_publisher;

    nt::DoubleTopic rotate_pid_voltage_topic;
    nt::DoublePublisher rotate_pid_voltage_publisher;

    nt::DoubleTopic rotate_error_topic;
    nt::DoublePublisher rotate_error_publisher;



};

}
