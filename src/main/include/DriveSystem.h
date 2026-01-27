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

    static inline const std::string MODULE_NT_PREFIX = "DriveSystem/SwerveModules/";
private:
    std::shared_ptr<RobotContext> context;

    std::shared_ptr<HighVoltage::MK5NSwerveModule> front_left_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> front_right_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> back_left_module;
    std::shared_ptr<HighVoltage::MK5NSwerveModule> back_right_module;

    std::unique_ptr<HighVoltage::SwerveModuleCordinator> swerve_cordinator;
};