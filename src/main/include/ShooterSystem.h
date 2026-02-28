#pragma once
#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <rev/ServoHub.h>

#include "RobotConfig.h"
#include "RobotContext.h"
#include "CommonConfig.h"
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
    enum class ShooterMode
    {
        STOP,
        SPEED,
        VOLTAGE
    };
    static inline const std::string SHOOTER_TABLE_NAME = "ShooterSystem";

    ShooterSystem(std::shared_ptr<RobotContext> context);
    ShooterSystem() = delete;

    void setShooterMode(ShooterMode mode);
    void setShooterVoltages(units::volt_t left_voltage, units::volt_t right_voltage);
    void setShooterTargetVelocities(units::revolutions_per_minute_t left_velocity, units::revolutions_per_minute_t right_velocity);
    void setLoaderMode(LoaderMode mode);
    units::revolutions_per_minute_t getLeftShooterVelocity();
    units::revolutions_per_minute_t getRightShooterVelocity();

    void Periodic() override;
    std::string loaderModeToString(LoaderMode mode);
    std::string shooterModeToString(ShooterMode mode);

private:
    void updateNetworkTable();

    void doShooterVoltageControl();
    void doShooterStop();
    void doShooterSpeedControl();

    void doLoaderStop();
    void doLoaderLoad();
    void doLoaderUnload();

    std::shared_ptr<RobotContext> context;
    LoaderMode loader_mode{LoaderMode::STOP};
    ShooterMode shooter_mode{ShooterMode::STOP};

    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> hopper_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> feed_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> left_shooter_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> right_shooter_motor;
    std::unique_ptr<rev::servohub::ServoHub> servo_controller;
    uint8_t left_servo_id;
    uint8_t right_servo_id;

    ctre::phoenix6::controls::VelocityVoltage shooter_velocity_request{0_tps};
    units::volt_t hopper_voltage;
    units::volt_t feed_voltage;
    units::volt_t left_shooter_voltage{0_V};
    units::volt_t right_shooter_voltage{0_V};
    units::revolutions_per_minute_t left_shooter_target_velocity{0_rpm};
    units::revolutions_per_minute_t right_shooter_target_velocity{0_rpm};
    double shooter_gear_ratio;
    units::volt_t max_shooter_voltage;
    units::volt_t min_shooter_voltage;

    HighVoltage::NetworkTableValue<double> nt_hopper_voltage{SHOOTER_TABLE_NAME + "/" + "hopper_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_feed_voltage{SHOOTER_TABLE_NAME + "/" + "feed_voltage", 0.0};
    HighVoltage::NetworkTableValue<std::string> nt_loader_mode{SHOOTER_TABLE_NAME + "/" + "loader_mode", "STOP"};
    HighVoltage::NetworkTableValue<std::string> nt_shooter_mode{SHOOTER_TABLE_NAME + "/" + "shooter_mode", "STOP"};
    HighVoltage::NetworkTableValue<double> nt_shooter_velocity{SHOOTER_TABLE_NAME + "/" + "shooter_velocity", 0.0};
    HighVoltage::NetworkTableValue<double> nt_shooter_gear_ratio{SHOOTER_TABLE_NAME + "/" + "shooter_gear_ratio", 0.0};
    HighVoltage::NetworkTableValue<double> nt_left_shooter_velocity{SHOOTER_TABLE_NAME + "/" + "left_shooter_velocity", 0.0};
    HighVoltage::NetworkTableValue<double> nt_right_shooter_velocity{SHOOTER_TABLE_NAME + "/" + "right_shooter_velocity", 0.0};
    HighVoltage::NetworkTableValue<double> nt_left_shooter_voltage{SHOOTER_TABLE_NAME + "/" + "left_shooter_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_right_shooter_voltage{SHOOTER_TABLE_NAME + "/" + "right_shooter_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_left_shooter_target_velocity{SHOOTER_TABLE_NAME + "/" + "left_shooter_target_velocity", 0.0};
    HighVoltage::NetworkTableValue<double> nt_right_shooter_target_velocity{SHOOTER_TABLE_NAME + "/" + "right_shooter_target_velocity", 0.0};

    //PID network table values (used to live-tune the TalonFX closed-loop gains)
    HighVoltage::NetworkTableValue<double> nt_shooter_kP{SHOOTER_TABLE_NAME + "/" + "shooter_kP", 0.0};
    HighVoltage::NetworkTableValue<double> nt_shooter_kI{SHOOTER_TABLE_NAME + "/" + "shooter_kI", 0.0};
    HighVoltage::NetworkTableValue<double> nt_shooter_kD{SHOOTER_TABLE_NAME + "/" + "shooter_kD", 0.0};

    //Network table values for recording data during testing
    HighVoltage::NetworkTableValue<std::vector<double>> nt_shooter_velocities{SHOOTER_TABLE_NAME + "/" + "shooter_velocities", std::vector<double>{0,0,0,0}};



};