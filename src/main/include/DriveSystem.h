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
#include "Keeko/ThreadedScript.h"

class DriveSystem: public frc2::SubsystemBase
{
public:
    enum DriveMode
    {
        MANUAL, 
        DRIVE_FF,
        TURN_FF,
        STOP,
        DRIVE_TO,
        ROTATE_TO
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
    void driveTo(frc::Pose2d pose);
    void driveToLua(double x, double y, double angle);
    void rotateTo(units::degree_t angle);
    void rotateToLua(double angle);
    units::degree_t getAngleFromTarget();
    double getAngleFromTargetLua();
    units::meter_t getDistanceFromTarget();
    double getDistanceFromTargetLua();
    units::degree_t getRobotAngleInDegrees();
    double getRobotAngleInDegreesLua();
    units::degree_t getRotationError();
    double getRotationErrorLua();
    void setOdometry(frc::Pose2d pose);
    void setOdometryLua(double x, double y, double angle);

    void addToLuaState(Keeko::ThreadedScript& script);

    static inline const std::string MODULE_NT_PREFIX = "DriveSystem/SwerveModules/";
    static inline constexpr double MANUAL_TRANSLATE_DEADZONE = 0.05;
    static inline constexpr double MANUAL_ROTATE_DEADZONE = 0.05;
    static inline const std::string ORIENTATION_TABLE_NAME = "Robot_Orientation";
    static inline const std::string X_KEY = "X";
    static inline const std::string Y_KEY = "Y";
    static inline const std::string ANGLE_KEY = "Angle";
    static inline const std::string TARGET_X_KEY = "Target_X";
    static inline const std::string TARGET_Y_KEY = "Target_Y";
    static inline const std::string TARGET_ANGLE_KEY = "Target_Angle";

private:
    void updateNetworkTables();
    void doManualDrive();
    void doStop();
    void doDriveFF();
    void doTurnFF();
    void doDriveTo();
    void doRotateTo();
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
    frc::Pose2d target_pose;
    std::shared_ptr<HighVoltage::SmartPID> angle_correction_pid;
    double drive_to_proportional = 1.0;
    double rotate_to_proportional = 0.0105;

    std::shared_ptr<frc::SwerveDriveKinematics<4>> kinematics;
    std::shared_ptr<frc::SwerveDrivePoseEstimator<4>> odometry;
    frc::Pose2d current_pose;
    frc::Pose2d updateOdometry();

    units::volt_t ff_voltage{0.0_V};
    ModuleName turn_ff_module_name{FRONT_LEFT};

    HighVoltage::NetworkTableValue<std::string> nt_drive_mode{"DriveSystem/DriveMode", driveModeToString(DriveMode::STOP)};
    HighVoltage::NetworkTableValue<std::vector<double>> nt_ff_data{"DriveSystem/FFData", {0.0, 0.0}};
    HighVoltage::NetworkTableValue<std::string> nt_turn_ff_module{"DriveSystem/TurnFFModule", moduleNameToString(FRONT_LEFT)};
    HighVoltage::NetworkTableValue<double> nt_drive_x{ORIENTATION_TABLE_NAME + "/" + X_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_drive_y{ORIENTATION_TABLE_NAME + "/" + Y_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_drive_angle{ORIENTATION_TABLE_NAME + "/" + ANGLE_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_target_drive_x{ORIENTATION_TABLE_NAME + "/" + TARGET_X_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_target_drive_y{ORIENTATION_TABLE_NAME + "/" + TARGET_Y_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_target_drive_angle{ORIENTATION_TABLE_NAME + "/" + TARGET_ANGLE_KEY, 0.0};
    
};