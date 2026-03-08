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
    // ---- Enums ----
    enum DriveMode
    {
        STOP,
        MANUAL, 
        DRIVE_FF,
        TURN_FF,
        DRIVE_TO,
        ROTATE_TO,
        MANUAL_AA,
        DRIVE_TO_SHOOT_LOCATION
    };

    enum ModuleName
    {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    };

    // ---- Constants ----
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
    static inline constexpr units::degree_t LOCK_ON_TOLERANCE{3.0_deg};

    // ---- Constructor ----
    DriveSystem(std::shared_ptr<RobotContext> context);
    DriveSystem() = delete;

    // ---- Subsystem Override ----
    void Periodic() override;

    // ---- Drive Commands ----
    void setDrive(double x, double y, double rotate);
    void setManualAA(double x, double y);
    void setDriveToShootLocation();
    void stopDrive();
    void setDriveFF(units::volt_t voltage);
    void setTurnFF(units::volt_t voltage, ModuleName module_name);
    void driveTo(frc::Pose2d pose);
    void rotateTo(units::degree_t angle);
    void setMaxSpeed(units::meters_per_second_t speed);
    void setMaxSpeedLua(double speed);
    void setMinPercentage(double percentage);

    // ---- Navigation ----
    units::degree_t getAngleFromTarget();
    units::meter_t getDistanceFromTarget();
    units::degree_t getRobotAngleInDegrees();
    units::degree_t getRotationError();
    units::degree_t getAngleFromTargetPosition(frc::Pose2d target_point);
    units::meter_t getDistanceFromTargetPosition(frc::Pose2d target_point);
    bool isLockedOn();

    // ---- Odometry ----
    void setOdometry(frc::Pose2d pose);
    void setOdometryFromVision();

    // ---- Enum Helpers ----
    DriveMode getDriveMode() { return drive_mode; }
    std::string driveModeToString(DriveMode mode);
    DriveMode stringToDriveMode(const std::string& mode_str);
    ModuleName getTurnFFModuleName() { return turn_ff_module_name; }
    std::string moduleNameToString(ModuleName name);
    ModuleName stringToModuleName(const std::string& name_str);

    // ---- Lua Bindings ----
    void addToLuaState(Keeko::ThreadedScript& script);
    void driveToLua(double x, double y, double angle);
    void rotateToLua(double angle);
    double getAngleFromTargetLua();
    double getDistanceFromTargetLua();
    double getRobotAngleInDegreesLua();
    double getRotationErrorLua();
    void setOdometryLua(double x, double y, double angle);

private:
    // ---- Drive Mode Handlers ----
    void doStop();
    void doManualDrive();
    void doManualAA();
    void doDriveFF();
    void doTurnFF();
    void doDriveTo();
    void doRotateTo();

    // ---- Internal Helpers ----
    frc::Pose2d updateOdometry();
    void updateNetworkTables();

    // ---- Core State ----
    std::shared_ptr<RobotContext> context;
    DriveMode drive_mode{DriveMode::STOP};

    // ---- Swerve Modules ----
    std::shared_ptr<HighVoltage::MK5NSwerveModule> front_left_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> front_right_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> back_left_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> back_right_module;
    std::unique_ptr<HighVoltage::SwerveModuleCordinator> swerve_coordinator;

    // ---- Manual Drive State ----
    double manual_drive_x{0.0};
    double manual_drive_y{0.0};
    double manual_rotate{0.0};

    // ---- Navigation State ----
    units::degree_t target_angle = 0_deg;
    frc::Pose2d target_pose;
    std::shared_ptr<HighVoltage::SmartPID> angle_correction_pid;
    double drive_to_proportional = 1.0;
    double rotate_to_proportional = 0.0115;
    units::degree_t angle_to_hub = 0.0_deg;
    units::meter_t distance_to_hub = 0.0_m;
    double min_auto_percentage = 0.0;

    // ---- Odometry ----
    std::shared_ptr<frc::SwerveDriveKinematics<4>> kinematics;
    std::shared_ptr<frc::SwerveDrivePoseEstimator<4>> odometry;
    frc::Pose2d current_pose;

    // ---- Feed Forward State ----
    units::volt_t ff_voltage{0.0_V};
    ModuleName turn_ff_module_name{FRONT_LEFT};

    // ---- Network Tables ----
    HighVoltage::NetworkTableValue<std::string> nt_drive_mode{"DriveSystem/DriveMode", driveModeToString(DriveMode::STOP)};
    HighVoltage::NetworkTableValue<std::vector<double>> nt_ff_data{"DriveSystem/FFData", {0.0, 0.0}};
    HighVoltage::NetworkTableValue<std::string> nt_turn_ff_module{"DriveSystem/TurnFFModule", moduleNameToString(FRONT_LEFT)};
    HighVoltage::NetworkTableValue<double> nt_drive_x{ORIENTATION_TABLE_NAME + "/" + X_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_drive_y{ORIENTATION_TABLE_NAME + "/" + Y_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_drive_angle{ORIENTATION_TABLE_NAME + "/" + ANGLE_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_target_drive_x{ORIENTATION_TABLE_NAME + "/" + TARGET_X_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_target_drive_y{ORIENTATION_TABLE_NAME + "/" + TARGET_Y_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_target_drive_angle{ORIENTATION_TABLE_NAME + "/" + TARGET_ANGLE_KEY, 0.0};
    HighVoltage::NetworkTableValue<double> nt_angle_to_hub{ORIENTATION_TABLE_NAME + "/" + "angle_to_hub", 0.0};
    HighVoltage::NetworkTableValue<double> nt_distance_to_hub{ORIENTATION_TABLE_NAME + "/" + "distance_to_hub", 0.0};
    HighVoltage::NetworkTableValue<bool> nt_locked_on{"DriveSystem/locked_on",false};
};