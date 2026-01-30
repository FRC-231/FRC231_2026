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


class DriveSystem: public frc2::SubsystemBase
{
public:
    DriveSystem(std::shared_ptr<RobotContext> context);
    DriveSystem() = delete;
    void Periodic() override;
    void setDrive(double x, double y, double rotate);
    void stopDrive();
    units::degree_t getRobotAngleInDegrees();
    static inline const std::string MODULE_NT_PREFIX = "DriveSystem/SwerveModules/";
    static inline constexpr double MANUAL_TRANSLATE_DEADZONE = 0.05;
    static inline constexpr double MANUAL_ROTATE_DEADZONE = 0.05;
    enum DriveMode
    {
        MANUAL, 
        DRIVE_FF, 
        TURN_FF, 
        STOP
    };
    std::string driveModeToString(DriveMode mode);

private:
    void doManualDrive();
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
};