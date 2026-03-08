#pragma once
#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include "ctre/phoenix6/StatusSignal.hpp"
#include "Keeko/ThreadedScript.h"

#include "RobotConfig.h"
#include "RobotContext.h"

#include "NetworkTableValue.h"

class IntakeSystem: public frc2::SubsystemBase
{
public:
    // ---- Enums ----
    enum class FeedMode
    {
        STOP,
        INTAKE,
        OUTTAKE
    };

    enum class PivotMode
    {
        STOP,
        UP,
        DOWN
    };

    // ---- Constants ----
    static inline const std::string INTAKE_TABLE_NAME = "IntakeSystem";

    // ---- Constructor ----
    IntakeSystem(std::shared_ptr<RobotContext> context);
    IntakeSystem() = delete;

    // ---- Subsystem Override ----
    void Periodic() override;

    // ---- Feed Commands ----
    void feedStop();
    void feedIntake();
    void feedOuttake();

    // ---- Pivot Commands ----
    void pivotStop();
    void pivotUp();
    void pivotDown();

    // ---- Getters ----
    units::degree_t getPivotAngle();

    // ---- Enum Helpers ----
    std::string feedModeToString(FeedMode mode);
    std::string pivotModeToString(PivotMode mode);

    void addToLuaState(Keeko::ThreadedScript& script);


private:
    // ---- Mode Handlers ----
    void doFeedStop();
    void doFeedIntake();
    void doFeedOuttake();
    void doPivotStop();
    void doPivotMove();

    // ---- Internal Helpers ----
    void updateNetworkTable();

    // ---- Core State ----
    std::shared_ptr<RobotContext> context;
    FeedMode feed_mode{FeedMode::STOP};
    PivotMode pivot_mode{PivotMode::STOP};

    // ---- Hardware ----
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> pivot_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> feed_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> pivot_encoder;

    // ---- Feed Config ----
    units::volt_t feed_intake_voltage;
    units::volt_t feed_outtake_voltage;

    // ---- Pivot Config ----
    units::volt_t pivot_max_voltage;
    units::volt_t pivot_min_voltage;
    units::degree_t pivot_max_angle;
    units::degree_t pivot_min_angle;
    units::degree_t pivot_up_angle;
    units::degree_t pivot_down_angle;
    units::degree_t pivot_encoder_offset;
    std::unique_ptr<frc::PIDController> pivot_pid;

    // ---- Network Tables ----
    HighVoltage::NetworkTableValue<std::string> nt_feed_mode{INTAKE_TABLE_NAME + "/" + "feed_mode", "STOP"};
    HighVoltage::NetworkTableValue<std::string> nt_pivot_mode{INTAKE_TABLE_NAME + "/" + "pivot_mode", "STOP"};
    HighVoltage::NetworkTableValue<double> nt_feed_intake_voltage{INTAKE_TABLE_NAME + "/" + "feed_intake_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_feed_outtake_voltage{INTAKE_TABLE_NAME + "/" + "feed_outtake_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_max_voltage{INTAKE_TABLE_NAME + "/" + "pivot_max_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_min_voltage{INTAKE_TABLE_NAME + "/" + "pivot_min_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_max_angle{INTAKE_TABLE_NAME + "/" + "pivot_max_angle", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_min_angle{INTAKE_TABLE_NAME + "/" + "pivot_min_angle", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_up_angle{INTAKE_TABLE_NAME + "/" + "pivot_up_angle", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_down_angle{INTAKE_TABLE_NAME + "/" + "pivot_down_angle", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_encoder_offset{INTAKE_TABLE_NAME + "/" + "pivot_encoder_offset", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_encoder_raw{INTAKE_TABLE_NAME + "/" + "pivot_encoder_raw", 0.0};

};