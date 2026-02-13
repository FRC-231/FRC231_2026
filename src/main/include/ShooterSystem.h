#pragma once
#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "RobotConfig.h"
#include "RobotContext.h"

#include "NetworkTableValue.h"

class ShooterSystem: public frc2::SubsystemBase
{
public:
    enum class LoaderMode
    {
        STOP,
        LOAD,
        UNLOAD
    };
    static inline const std::string SHOOTER_TABLE_NAME = "ShooterSystem";

    ShooterSystem(std::shared_ptr<RobotContext> context);
    ShooterSystem() = delete;

    void Periodic() override;
    std::string loaderModeToString(LoaderMode mode);

private:
    void updateNetworkTable();

    std::shared_ptr<RobotContext> context;
    LoaderMode loader_mode{LoaderMode::STOP};

    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> hopper_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> feed_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> left_shooter_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> right_shooter_motor;

    std::unique_ptr<frc::PIDController> shooter_pid;
    std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> shooter_ff;
    units::volt_t hopper_voltage;
    units::volt_t feed_voltage;

    HighVoltage::NetworkTableValue<double> nt_hopper_voltage{SHOOTER_TABLE_NAME + "/" + "hopper_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_feed_voltage{SHOOTER_TABLE_NAME + "/" + "feed_voltage", 0.0};
    HighVoltage::NetworkTableValue<std::string> nt_loader_mode{SHOOTER_TABLE_NAME + "/" + "loader_mode", "STOP"};
};