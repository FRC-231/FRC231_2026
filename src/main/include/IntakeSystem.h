#pragma once
#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "RobotConfig.h"
#include "RobotContext.h"

#include "MK5NSwerveModule.h"
#include "SwerveDriveCordinator.h"
#include "NetworkTableValue.h"
#include "SmartPID.h"
#include "Keeko/ThreadedScript.h"

class IntakeSystem: public frc2::SubsystemBase
{
public:
    IntakeSystem(std::shared_ptr<RobotContext> context);
    IntakeSystem() = delete;
    void Periodic() override;
    units::degree_t getPivotAngle(); 
    static inline const std::string INTAKE_TABLE_NAME = "IntakeSystem";

    enum class FeedMode
    {
        STOP,
        INTAKE,
        OUTTAKE
    };
    std::string feedModeToString(FeedMode mode);
    enum class PivotMode
    {
        STOP,
        UP,
        DOWN
    };
    std::string pivotModeToString(PivotMode mode);

    void feedStop();
    void feedIntake();
    void feedOuttake();
    void pivotStop();
    void pivotUp();
    void pivotDown();

private: 
    std::shared_ptr<RobotContext> context;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> pivot_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> feed_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> pivot_encoder;
    FeedMode feed_mode{FeedMode::STOP};
    PivotMode pivot_mode{PivotMode::STOP};
    units::volt_t feed_intake_voltage;
    units::volt_t feed_outtake_voltage;
    units::volt_t pivot_max_voltage;
    units::volt_t pivot_min_voltage;
    units::degree_t pivot_max_angle;
    units::degree_t pivot_min_angle;
    units::degree_t pivot_up_angle;
    units::degree_t pivot_down_angle;
    std::unique_ptr<frc::PIDController> pivot_pid;
    units::degree_t pivot_encoder_offset;

    void doFeedStop();
    void doPivotStop();
    void doFeedIntake();
    void doFeedOuttake();
    void doPivotMove();
    void updateNetworkTable();

    HighVoltage::NetworkTableValue<double> nt_feed_intake_voltage{INTAKE_TABLE_NAME + "/" + "feed_intake_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_feed_outtake_voltage{INTAKE_TABLE_NAME + "/" + "feed_outtake_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_max_voltage{INTAKE_TABLE_NAME + "/" + "pivot_max_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_min_voltage{INTAKE_TABLE_NAME + "/" + "pivot_min_voltage", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_max_angle{INTAKE_TABLE_NAME + "/" + "pivot_max_angle", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_min_angle{INTAKE_TABLE_NAME + "/" + "pivot_min_angle", 0.0};
    HighVoltage::NetworkTableValue<std::string> nt_feed_mode{INTAKE_TABLE_NAME + "/" + "feed_mode", "STOP"};
    HighVoltage::NetworkTableValue<std::string> nt_pivot_mode{INTAKE_TABLE_NAME + "/" + "pivot_mode", "STOP"};
    HighVoltage::NetworkTableValue<double> nt_pivot_up_angle{INTAKE_TABLE_NAME + "/" + "pivot_up_angle", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_down_angle{INTAKE_TABLE_NAME + "/" + "pivot_down_angle", 0.0};
    HighVoltage::NetworkTableValue<double> nt_pivot_encoder_offset{INTAKE_TABLE_NAME + "/" + "pivot_encoder_offset", 0.0};

};