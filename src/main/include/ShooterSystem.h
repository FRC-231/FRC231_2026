#pragma once
#include <memory>
#include <functional>

#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>
#include <units/length.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <rev/ServoHub.h>

#include "RobotConfig.h"
#include "RobotContext.h"
#include "CommonConfig.h"
#include "NetworkTableValue.h"
#include "Keeko/ThreadedScript.h"

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
        VOLTAGE,
        AUTO,
        MANUAL_AUTO,
        IDLE,
        SNOW_BLOW
    };
    static inline const std::string SHOOTER_TABLE_NAME = "ShooterSystem";

    ShooterSystem(std::shared_ptr<RobotContext> context);
    ShooterSystem() = delete;

    void setShooterMode(ShooterMode mode);
    void stopShooter();
    void autoShoot();
    void snowBlow();
    void setShooterVoltages(units::volt_t left_voltage, units::volt_t right_voltage);
    void setShooterVoltagesLua(double left_voltage, double right_voltage);
    void setShooterTargetVelocities(units::revolutions_per_minute_t left_velocity, units::revolutions_per_minute_t right_velocity);
    void setShooterTargetVelocitiesLua(double left_velocity, double right_velocity);
    void setLoaderMode(LoaderMode mode);
    void loaderStop();
    void loaderLoad();
    void loaderUnload();
    bool isAtSpeed();
    void setManualShooterAdjustmentValue(double adjustment);
    units::revolutions_per_minute_t getLeftShooterVelocity();
    units::revolutions_per_minute_t getRightShooterVelocity();
    void setDistanceUpdateCallback(std::function<units::meter_t()> callback);
    std::map<units::meter_t, units::revolutions_per_minute_t> getDistanceToShooterVelocityMap(){return distance_to_shooter_velocity;};

    void Periodic() override;
    std::string loaderModeToString(LoaderMode mode);
    std::string shooterModeToString(ShooterMode mode);

    /// Sets the NT telemetry throttle interval.
    /// 0 or 1 = publish every loop (~50Hz), N = publish every Nth loop.
    void setNTThrottleInterval(int interval) { shooter_nt_throttle_interval_ = interval; }

    void addToLuaState(Keeko::ThreadedScript& script);

private:
    void updateNetworkTable();

    void doShooterVoltageControl();
    void doShooterStop();
    void doShooterSpeedControl();
    void doShooterAutoControl();
    void doShooterManualAutoControl();
    void doShooterIdleControl();
    void doShooterSnowBlowControl();

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
    units::volt_t hopper_wiggle_low_voltage;
    double hopper_wiggle_period;
    units::volt_t feed_voltage;
    units::volt_t left_shooter_voltage{0_V};
    units::volt_t right_shooter_voltage{0_V};
    units::revolutions_per_minute_t left_shooter_target_velocity{0_rpm};
    units::revolutions_per_minute_t right_shooter_target_velocity{0_rpm};
    double shooter_gear_ratio;
    units::volt_t max_shooter_voltage;
    units::volt_t min_shooter_voltage;
    int servo_pulse_width = 1500;
    int servo_center_pulse_width = 1500;
    units::meter_t distance_to_target{0_m};
    std::function<units::meter_t()> distance_update_callback;
    units::revolutions_per_minute_t shooter_auto_adjustment{0_rpm};
    units::revolutions_per_minute_t backspin_amount{0_rpm};
    units::revolutions_per_minute_t backspin_adjustment{0_rpm};
    double shooter_angle_adjustment{0.0};
    std::map<units::meter_t, units::revolutions_per_minute_t> distance_to_shooter_velocity;
    std::map<units::meter_t, double> distance_to_pulsewidth;
    std::map<units::meter_t, units::revolutions_per_minute_t> distance_to_backspin;
    units::revolutions_per_minute_t shooter_manual_auto_speed{0_rpm};
    units::revolutions_per_minute_t at_speed_tolerance{0_rpm};
    units::revolutions_per_minute_t idle_mode_velocity{0_rpm};
    units::revolutions_per_minute_t snow_blow_target_rpm{0_rpm};
    int snow_blow_pulse_width{1500};
    units::revolutions_per_minute_t manual_shooter_adjustment_multiplier{0.0};
    units::revolutions_per_minute_t manual_shooter_adjustment{0.0};

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
    HighVoltage::NetworkTableValue<int> nt_servo_pulse_width{SHOOTER_TABLE_NAME + "/" + "servo_pulse_width", 0};
    HighVoltage::NetworkTableValue<double> nt_shooter_auto_adjustment{SHOOTER_TABLE_NAME + "/" + "shooter_auto_adjustment", 0.0};
    HighVoltage::NetworkTableValue<double> nt_backspin_amount{SHOOTER_TABLE_NAME + "/" + "backspin_amount", 0.0};
    HighVoltage::NetworkTableValue<double> nt_backspin_adjustment{SHOOTER_TABLE_NAME + "/" + "backspin_adjustment", 0.0};
    HighVoltage::NetworkTableValue<double> nt_shooter_angle_adjustment{SHOOTER_TABLE_NAME + "/" + "shooter_angle_adjustment", 0.0};
    HighVoltage::NetworkTableValue<double> nt_shooter_manual_auto_speed{SHOOTER_TABLE_NAME + "/" + "shooter_manual_auto_speed", 0.0};
    HighVoltage::NetworkTableValue<bool> nt_is_shooter_at_speed{SHOOTER_TABLE_NAME + "/" + "is_shooter_at_speed", false};
    HighVoltage::NetworkTableValue<double> nt_snow_blow_target_rpm{SHOOTER_TABLE_NAME + "/" + "snow_blow_target_rpm", 0.0};
    HighVoltage::NetworkTableValue<int> nt_snow_blow_pulse_width{SHOOTER_TABLE_NAME + "/" + "snow_blow_pulse_width", 0};

    //PID network table values (used to live-tune the TalonFX closed-loop gains)
    HighVoltage::NetworkTableValue<double> nt_shooter_kP{SHOOTER_TABLE_NAME + "/" + "shooter_kP", 0.0};
    HighVoltage::NetworkTableValue<double> nt_shooter_kI{SHOOTER_TABLE_NAME + "/" + "shooter_kI", 0.0};
    HighVoltage::NetworkTableValue<double> nt_shooter_kD{SHOOTER_TABLE_NAME + "/" + "shooter_kD", 0.0};

    //Network table values for recording data during testing
    HighVoltage::NetworkTableValue<std::vector<double>> nt_shooter_velocities{SHOOTER_TABLE_NAME + "/" + "shooter_velocities", std::vector<double>{0,0,0,0}};

    int servo_diag_counter_{0};
    int shooter_nt_counter_{0};
    int shooter_nt_throttle_interval_{0};

};