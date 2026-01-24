#pragma once
#include <memory>

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "RobotConfig.h"
#include "RobotContext.h"

class DriveSystem: public frc2::SubsystemBase
{
public:
    DriveSystem(std::shared_ptr<RobotContext> context);
    DriveSystem() = delete;
    void Periodic() override;
private:
    std::shared_ptr<RobotContext> context;
};