#pragma once
#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "RobotConfig.h"
#include "RobotContext.h"

#include "MK5NSwerveModule.h"
#include "SwerveDriveCordinator.h"
#include "NetworkTableValue.h"
#include "SmartPID.h"


class DriveSystem: public frc2::SubsystemBase
{
public:
    enum DriveMode
    {
        MANUAL, 
        DRIVE_FF,
        TURN_FF,
        STOP
    };
    std::string driveModeToString(DriveMode mode);
    DriveMode getDriveMode() { return drive_mode; }
    DriveMode stringToDriveMode(const std::string& mode_str);

    enum ModuleName
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };
    std::string moduleNameToString(ModuleName name);
    ModuleName stringToModuleName(const std::string& name_str);
    ModuleName getTurnFFModuleName() { return turn_ff_module_name; }
    

    DriveSystem(std::shared_ptr<RobotContext> context);
    DriveSystem() = delete;
    void Periodic() override;
    void setDrive(double x, double y, double rotate);
    void stopDrive();
    void setDriveFF(units::volt_t voltage);
    void setTurnFF(units::volt_t voltage, ModuleName module_name);
    units::degree_t getRobotAngleInDegrees();
    static inline const std::string MODULE_NT_PREFIX = "DriveSystem/SwerveModules/";
    static inline constexpr double MANUAL_TRANSLATE_DEADZONE = 0.05;
    static inline constexpr double MANUAL_ROTATE_DEADZONE = 0.05;
    

private:
    void updateNetworkTables();
    void doManualDrive();
    void doStop();
    void doDriveFF();
    void doTurnFF();
    std::shared_ptr<RobotContext> context;
    DriveMode drive_mode{DriveMode::STOP};

    std::shared_ptr<HighVoltage::MK5NSwerveModule> front_left_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> front_right_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> back_left_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> back_right_module;
    std::unique_ptr<HighVoltage::SwerveModuleCordinator> swerve_cordinator;
    
    double manual_drive_x{0.0};
    double manual_drive_y{0.0};
    double manual_rotate{0.0};
    units::degree_t target_angle = 0_deg;
    std::shared_ptr<HighVoltage::SmartPID> angle_correction_pid;

    std::shared_ptr<frc::SwerveDriveKinematics<4>> kinematics;
    std::shared_ptr<frc::SwerveDrivePoseEstimator<4>> odometry;
    frc::Pose2d current_pose;
    frc::Pose2d updateOdometry();

    units::volt_t ff_voltage{0.0_V};
    ModuleName turn_ff_module_name{FRONT_LEFT};

    HighVoltage::NetworkTableValue<std::string> nt_drive_mode{"DriveSystem/DriveMode", driveModeToString(DriveMode::STOP)};
    HighVoltage::NetworkTableValue<std::vector<double>> nt_ff_data{"DriveSystem/FFData", {0.0, 0.0}};
    HighVoltage::NetworkTableValue<std::string> nt_turn_ff_module{"DriveSystem/TurnFFModule", moduleNameToString(FRONT_LEFT)};

};