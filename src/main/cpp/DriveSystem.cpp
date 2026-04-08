#include "DriveSystem.h"
#include <iostream>
#include <numbers>

DriveSystem::DriveSystem(std::shared_ptr<RobotContext> context):
    context(context)
{
    //Create swerve modules
    const auto& drive_config = context->getConfig()->drive_config;

    front_left_module = std::make_shared<HighVoltage::MK5NSwerveModule>(
        MODULE_NT_PREFIX + "front_left",
        drive_config.front_left_module
    );
    front_right_module = std::make_shared<HighVoltage::MK5NSwerveModule>(
        MODULE_NT_PREFIX + "front_right",
        drive_config.front_right_module
    );
    back_left_module = std::make_shared<HighVoltage::MK5NSwerveModule>(
        MODULE_NT_PREFIX + "back_left",
        drive_config.back_left_module
    );
    back_right_module = std::make_shared<HighVoltage::MK5NSwerveModule>(
        MODULE_NT_PREFIX + "back_right",
        drive_config.back_right_module
    );
    angle_correction_pid = std::make_shared<HighVoltage::SmartPID>(
        "angle_correction_pid",
        drive_config.angle_correction_pid
    );

    swerve_coordinator = std::make_unique<HighVoltage::SwerveModuleCordinator>(
        front_left_module,
        front_right_module,
        back_left_module,
        back_right_module,
        0.6096, //TODO: Use config values
        0.6096 //TODO: Use config values
    );
    swerve_coordinator->setMaxSpeed(5.0_mps);
    swerve_coordinator->setFieldOriented(true);

    kinematics = std::make_shared<frc::SwerveDriveKinematics<4>>(
        frc::Translation2d{units::meter_t{context->getConfig()->drive_config.swerve_kinematics.front_left_position.x}, units::meter_t{context->getConfig()->drive_config.swerve_kinematics.front_left_position.y}},
        frc::Translation2d{units::meter_t{context->getConfig()->drive_config.swerve_kinematics.front_right_position.x}, units::meter_t{context->getConfig()->drive_config.swerve_kinematics.front_right_position.y}},
        frc::Translation2d{units::meter_t{context->getConfig()->drive_config.swerve_kinematics.back_left_position.x}, units::meter_t{context->getConfig()->drive_config.swerve_kinematics.back_left_position.y}},
        frc::Translation2d{units::meter_t{context->getConfig()->drive_config.swerve_kinematics.back_right_position.x}, units::meter_t{context->getConfig()->drive_config.swerve_kinematics.back_right_position.y}}
    );

    odometry = std::make_shared<frc::SwerveDrivePoseEstimator<4>>(
        *(kinematics.get()),
        frc::Rotation2d{units::degree_t(0.0)},
        std::array<frc::SwerveModulePosition,4>{
            front_left_module->getModulePosition(),
            front_right_module->getModulePosition(),
            back_left_module->getModulePosition(),
            back_right_module->getModulePosition(),
        },
        frc::Pose2d
        {
            units::meter_t{0.0},
            units::meter_t{0.0},
            units::degree_t{0.0}
        }
    );
    drive_to_proportional = drive_config.drive_to_proportional;
    auto_aim_offset = units::degree_t{drive_config.auto_aim_offset};

    // Initialize the holonomic controller for trajectory following.
    // X and Y PID controllers correct positional error (meters).
    // The profiled PID controller corrects rotational error (radians).
    frc::ProfiledPIDController<units::radians> theta_controller{
        3.0, 0.0, 0.0,
        frc::TrapezoidProfile<units::radians>::Constraints{
            units::radians_per_second_t{3.14},
            units::unit_t<units::compound_unit<units::radians, units::inverse<units::squared<units::second>>>>{3.14}
        }
    };
    theta_controller.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
    holonomic_controller = std::make_unique<frc::HolonomicDriveController>(
        frc::PIDController{1.0, 0.0, 0.0},  // X position PID
        frc::PIDController{1.0, 0.0, 0.0},  // Y position PID
        theta_controller
    );

    nt_drive_mode.setOnUpdate([this](const std::string& mode_str)
    {
        std::cout << "Drive mode update received: " << mode_str << std::endl;
        if(mode_str == driveModeToString(this->drive_mode))
        {
            return;
        }
        this->drive_mode = stringToDriveMode(mode_str);
        std::cout << "Drive mode set to " << driveModeToString(this->drive_mode) << std::endl;
    });
    nt_turn_ff_module.setOnUpdate([this](const std::string& module_str)
    {
        std::cout << "Turn FF module update received: " << module_str << std::endl;
        if(module_str == moduleNameToString(this->turn_ff_module_name))
        {
            return;
        }
        this->turn_ff_module_name = stringToModuleName(module_str);
        std::cout << "Turn FF module set to " << moduleNameToString(this->turn_ff_module_name) << std::endl;
    });
    nt_auto_aim_offset.setOnUpdate([this](double offset)
    {
        std::cout << "Auto aim offset update received: " << offset << std::endl;
        this->auto_aim_offset = units::degree_t{offset};
        std::cout << "Auto aim offset set to " << this->auto_aim_offset.value() << " degrees" << std::endl;
    });
    nt_auto_aim_offset.setValue(drive_config.auto_aim_offset);

}

// ==== Subsystem Override ====

void DriveSystem::Periodic()
{
    current_pose = updateOdometry();
    swerve_coordinator->setFieldAngle(getRobotAngleInDegrees());
    angle_to_hub = getAngleFromTargetPosition(RobotContext::HUB_POSITION);
    distance_to_hub = getDistanceFromTargetPosition(RobotContext::HUB_POSITION);
    switch(drive_mode)
    {
        case STOP:
        {
            doStop();
        }break;

        case MANUAL:
        {
            doManualDrive();
        }break;

        case MANUAL_AA:
        {
            doManualAA();
        }break;

        case MANUAL_FACE_BACK:
        {
            doManualFaceBack();
        }break;

        case DRIVE_FF:
        {
            doDriveFF();
        }break;

        case TURN_FF:
        {
            doTurnFF();
        }break;

        case DRIVE_TO:
        {
            doDriveTo();
        }break;

        case ROTATE_TO:
        {
            doRotateTo();
        }break;

        case FOLLOW_TRAJECTORY:
        {
            doFollowTrajectory();
        }break;

        default:
        {
            doStop();
        }break;
    }
    updateNetworkTables();
}

// ==== Drive Commands ====

void DriveSystem::setDrive(double x, double y, double rotate)
{
    drive_mode = DriveMode::MANUAL;
    manual_drive_x = x;
    manual_drive_y = y;
    manual_rotate = rotate;
}

void DriveSystem::setManualAA(double x, double y)
{
    drive_mode = DriveMode::MANUAL_AA;
    manual_drive_x = x;
    manual_drive_y = y;
}

void DriveSystem::setManualFaceBack(double x, double y)
{
    drive_mode = DriveMode::MANUAL_FACE_BACK;
    manual_drive_x = x;
    manual_drive_y = y;
}

void DriveSystem::setAutoAA()
{
    drive_mode = DriveMode::MANUAL_AA;
    manual_drive_x = 0.0;
    manual_drive_y = 0.0;
}

void DriveSystem::setDriveToShootLocation()
{
    drive_mode = DriveMode::DRIVE_TO_SHOOT_LOCATION;
}

void DriveSystem::stopDrive()
{
    drive_mode = STOP;
}

void DriveSystem::setDriveFF(units::volt_t voltage)
{
    drive_mode = DriveMode::DRIVE_FF;
    ff_voltage = voltage;
}

void DriveSystem::setTurnFF(units::volt_t voltage, ModuleName module_name)
{
    drive_mode = DriveMode::TURN_FF;
    ff_voltage = voltage;
    turn_ff_module_name = module_name;
}

// ==== Navigation ====

void DriveSystem::driveTo(frc::Pose2d pose)
{
    target_pose = pose;
    drive_mode = DRIVE_TO;
    target_angle = target_pose.Rotation().Degrees();
}

void DriveSystem::rotateTo(units::degree_t angle)
{
    drive_mode = ROTATE_TO;
    target_angle = angle;
    target_pose = frc::Pose2d(target_pose.Translation(), frc::Rotation2d{angle});
}

void DriveSystem::followTrajectory(frc::Trajectory trajectory, frc::Rotation2d endRotation)
{
    current_trajectory = trajectory;
    trajectory_end_rotation = endRotation;
    trajectory_done = false;
    trajectory_timer.Reset();
    trajectory_timer.Start();
    drive_mode = FOLLOW_TRAJECTORY;
    std::cout << "Following trajectory: " << trajectory.TotalTime().value() << "s, "
              << trajectory.States().size() << " states" << std::endl;
}

bool DriveSystem::isTrajectoryDone()
{
    return trajectory_done;
}

void DriveSystem::setMaxSpeed(units::meters_per_second_t speed)
{
    swerve_coordinator->setMaxSpeed(speed);
}

void DriveSystem::setMaxSpeedLua(double speed)
{
    setMaxSpeed(units::meters_per_second_t{speed});
}

void DriveSystem::setMinPercentage(double percentage)
{
    min_auto_percentage = percentage;
}

units::degree_t DriveSystem::getAngleFromTarget()
{
    auto delta_pose = target_pose - current_pose;
    double angle_in_rad = std::atan2(delta_pose.Y().value(), delta_pose.X().value());
    return units::angle::degree_t{HighVoltage::get180Range(HighVoltage::toDegrees(angle_in_rad))};
}

units::meter_t DriveSystem::getDistanceFromTarget()
{
    auto delta_pose = target_pose - current_pose;
    double x = delta_pose.X().value();
    double y = delta_pose.Y().value();
    return units::meter_t{std::sqrt(x*x+y*y)};
}

units::degree_t DriveSystem::getRobotAngleInDegrees()
{
    return cached_robot_angle_;
}

units::degree_t DriveSystem::getRotationError()
{
    units::degree_t angle_error = target_angle - getRobotAngleInDegrees();
    angle_error = HighVoltage::get180Range(angle_error);
    angle_error *= -1;
    return angle_error;
}

units::degree_t DriveSystem::getAngleFromTargetPosition(frc::Pose2d target_point)
{
    // Calculate the vector from current position to target position
    double delta_x = target_point.X().value() - current_pose.X().value();
    double delta_y = target_point.Y().value() - current_pose.Y().value();
    
    // Calculate the angle in field coordinates (0° = +X axis, 90° = +Y axis)
    double angle_in_rad = std::atan2(delta_y, delta_x);
    
    // Convert to degrees and normalize to [-180, 180]
    return units::degree_t{HighVoltage::get180Range(HighVoltage::toDegrees(angle_in_rad))};
}

units::meter_t DriveSystem::getDistanceFromTargetPosition(frc::Pose2d target_point)
{
    double delta_x = target_point.X().value() - current_pose.X().value();
    double delta_y = target_point.Y().value() - current_pose.Y().value();
    return units::meter_t{std::sqrt(delta_x * delta_x + delta_y * delta_y)};
}

bool DriveSystem::isLockedOn()
{
    units::degree_t current_angle = getRobotAngleInDegrees();
    units::degree_t angle_error = angle_to_hub - current_angle;
    angle_error = HighVoltage::get180Range(angle_error);
    if(std::abs(angle_error.value()) > 90)
    {
        angle_error -= 180_deg;
        angle_error = HighVoltage::get180Range(angle_error);
    }
    return std::abs(angle_error.value()) < LOCK_ON_TOLERANCE.value();

}

// ==== Odometry ====

void DriveSystem::setOdometry(frc::Pose2d pose)
{
    context->setStartAngle(context->getPigeon().GetYaw().GetValue() - pose.Rotation().Degrees());
    odometry->ResetPosition(frc::Rotation2d(getRobotAngleInDegrees()),
    std::array<frc::SwerveModulePosition,4>{
            front_left_module->getModulePosition(),
            front_right_module->getModulePosition(),
            back_left_module->getModulePosition(),
            back_right_module->getModulePosition(),
        },
        pose
    );
}

void DriveSystem::setOdometryFromVision()
{
    frc::Pose2d vision_pose = context->getVision().getCameraPose();
    if(vision_pose.X().value() == 0.0 && vision_pose.Y().value() == 0.0)
    {
        std::cout << "setOdometryFromVision: No valid vision data available" << std::endl;
        return;
    }
    std::cout << "setOdometryFromVision: Setting odometry to ("
              << vision_pose.X().value() << ", "
              << vision_pose.Y().value() << ", "
              << vision_pose.Rotation().Degrees().value() << ")" << std::endl;
    setOdometry(vision_pose);
}

// ==== Enum Helpers ====

std::string DriveSystem::driveModeToString(DriveMode mode)
{
    switch(mode)
    {
        case STOP:
            return "STOP";

        case MANUAL:
            return "MANUAL";
        
        case DRIVE_FF:
            return "DRIVE_FF";

        case TURN_FF:
            return "TURN_FF";

        case DRIVE_TO:
            return "DRIVE_TO";

        case MANUAL_AA:
            return "MANUAL_AA";

        case MANUAL_FACE_BACK:
            return "MANUAL_FACE_BACK";

        case FOLLOW_TRAJECTORY:
            return "FOLLOW_TRAJECTORY";
    }
    return "UNKNOWN";
}

DriveSystem::DriveMode DriveSystem::stringToDriveMode(const std::string& mode_str)
{
    if(mode_str == "STOP")
    {
        return STOP;
    }
    else if(mode_str == "MANUAL")
    {
        return MANUAL;
    }
    else if(mode_str == "DRIVE_FF")
    {
        return DRIVE_FF;
    }
    else if(mode_str == "TURN_FF")
    {
        return TURN_FF;
    }
    else if(mode_str == "DRIVE_TO")
    {
        return DRIVE_TO;
    }
    else if(mode_str == "MANUAL_AA")
    {
        return MANUAL_AA;
    }
    else if(mode_str == "MANUAL_FACE_BACK")
    {
        return MANUAL_FACE_BACK;
    }
    else if(mode_str == "DRIVE_TO_SHOOT_LOCATION")
    {
        return DRIVE_TO_SHOOT_LOCATION;
    }
    else if(mode_str == "FOLLOW_TRAJECTORY")
    {
        return FOLLOW_TRAJECTORY;
    }
    return STOP;
}

std::string DriveSystem::moduleNameToString(ModuleName name)
{
    switch(name)
    {
        case FRONT_LEFT:
            return "FRONT_LEFT";

        case FRONT_RIGHT:
            return "FRONT_RIGHT";

        case BACK_LEFT:
            return "BACK_LEFT";

        case BACK_RIGHT:
            return "BACK_RIGHT";
    }
    return "UNKNOWN";
}

DriveSystem::ModuleName DriveSystem::stringToModuleName(const std::string& name_str)
{
    if(name_str == "FRONT_LEFT")
    {
        return FRONT_LEFT;
    }
    else if(name_str == "FRONT_RIGHT")
    {
        return FRONT_RIGHT;
    }
    else if(name_str == "BACK_LEFT")
    {
        return BACK_LEFT;
    }
    else if(name_str == "BACK_RIGHT")
    {
        return BACK_RIGHT;
    }
    return FRONT_LEFT;
}

// ==== Lua Bindings ====

void DriveSystem::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
        .beginClass<DriveSystem> ("DriveSystem")
            .addFunction("driveTo", &DriveSystem::driveToLua)
            .addFunction("stop", &DriveSystem::stopDrive)
            .addFunction("stopDrive", &DriveSystem::stopDrive)
            .addFunction("getAngleFromTarget", &DriveSystem::getAngleFromTargetLua)
            .addFunction("getDistanceFromTarget", &DriveSystem::getDistanceFromTargetLua)
            .addFunction("getRobotAngleInDegrees", &DriveSystem::getRobotAngleInDegreesLua)
            .addFunction("setOdometry", &DriveSystem::setOdometryLua)
            .addFunction("rotateTo", &DriveSystem::rotateToLua)
            .addFunction("getRotationError", &DriveSystem::getRotationErrorLua)
            .addFunction("setMaxSpeed", &DriveSystem::setMaxSpeedLua)
            .addFunction("setMinPercentage", &DriveSystem::setMinPercentage)
            .addFunction("setAutoAA", &DriveSystem::setAutoAA)
            .addFunction("setManualFaceBack", &DriveSystem::setManualFaceBack)
            .addFunction("followTrajectory", &DriveSystem::followTrajectoryLua)
            .addFunction("followPath", &DriveSystem::followPathLua)
            .addFunction("isTrajectoryDone", &DriveSystem::isTrajectoryDoneLua)
        .endClass();
    script.addInstance("drive_system", this);
}

void DriveSystem::driveToLua(double x, double y, double angle)
{
    driveTo(
        frc::Pose2d{
            units::meter_t{x},
            units::meter_t{y},
            frc::Rotation2d{units::degree_t{angle}}
    });
}

void DriveSystem::rotateToLua(double angle)
{
    rotateTo(units::degree_t{angle});
}

double DriveSystem::getAngleFromTargetLua()
{
    return getAngleFromTarget().value();
}

double DriveSystem::getDistanceFromTargetLua()
{
    return getDistanceFromTarget().value();
}

double DriveSystem::getRobotAngleInDegreesLua()
{
    return getRobotAngleInDegrees().value();
}

double DriveSystem::getRotationErrorLua()
{
    return getRotationError().value();
}

void DriveSystem::setOdometryLua(double x, double y, double angle)
{
    setOdometry(
        frc::Pose2d{
            units::meter_t{x},
            units::meter_t{y},
            frc::Rotation2d{units::degree_t{angle}}
        }
    );
}

void DriveSystem::followTrajectoryLua(double maxAccel,
                                       double startX, double startY, double startAngle,
                                       std::vector<std::vector<double>> interiorPoints,
                                       double endX, double endY, double endAngle)
{
    // Use the max speed already set via setMaxSpeed() on the swerve coordinator
    frc::TrajectoryConfig config(
        swerve_coordinator->getMaxSpeed(),
        units::meters_per_second_squared_t{maxAccel}
    );

    // Build start pose
    frc::Pose2d startPose{
        units::meter_t{startX},
        units::meter_t{startY},
        frc::Rotation2d{units::degree_t{startAngle}}
    };

    // Build interior waypoints from the Lua table (each element is {x, y})
    std::vector<frc::Translation2d> waypoints;
    for(const auto& point : interiorPoints)
    {
        if(point.size() >= 2)
        {
            waypoints.emplace_back(units::meter_t{point[0]}, units::meter_t{point[1]});
        }
        else
        {
            std::cerr << "followTrajectoryLua: interior point needs at least 2 values (x, y), skipping" << std::endl;
        }
    }

    // Build end pose
    frc::Pose2d endPose{
        units::meter_t{endX},
        units::meter_t{endY},
        frc::Rotation2d{units::degree_t{endAngle}}
    };

    // Generate the trajectory
    auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose, waypoints, endPose, config
    );

    // Start following it. The end rotation controls which direction the robot faces
    // while driving (independent of the path direction).
    followTrajectory(trajectory, frc::Rotation2d{units::degree_t{endAngle}});
}

void DriveSystem::followPathLua(double maxAccel,
                                 std::vector<std::vector<double>> points,
                                 double endAngle)
{
    // Need at least one point (the end point)
    if(points.empty())
    {
        std::cerr << "followPathLua: need at least one point (the end point)" << std::endl;
        return;
    }

    // The last point in the table is the end pose (x, y)
    auto& lastPoint = points.back();
    if(lastPoint.size() < 2)
    {
        std::cerr << "followPathLua: end point needs at least 2 values (x, y)" << std::endl;
        return;
    }

    // All points except the last are interior waypoints
    std::vector<frc::Translation2d> waypoints;
    for(size_t i = 0; i < points.size() - 1; i++)
    {
        if(points[i].size() >= 2)
        {
            waypoints.emplace_back(units::meter_t{points[i][0]}, units::meter_t{points[i][1]});
        }
        else
        {
            std::cerr << "followPathLua: waypoint needs at least 2 values (x, y), skipping" << std::endl;
        }
    }

    // Compute the start pose travel heading: direction from current position
    // toward the first point in the path. WPILib TrajectoryGenerator uses the
    // pose heading as the direction of travel, NOT the robot's facing direction.
    frc::Translation2d firstTarget = waypoints.empty()
        ? frc::Translation2d{units::meter_t{lastPoint[0]}, units::meter_t{lastPoint[1]}}
        : waypoints.front();
    double dx = firstTarget.X().value() - current_pose.X().value();
    double dy = firstTarget.Y().value() - current_pose.Y().value();
    frc::Rotation2d startHeading{units::radian_t{std::atan2(dy, dx)}};

    frc::Pose2d startPose{current_pose.X(), current_pose.Y(), startHeading};

    // Compute the end pose travel heading: direction from the last waypoint
    // (or current position if no waypoints) toward the end point.
    frc::Translation2d lastWaypoint = waypoints.empty()
        ? frc::Translation2d{current_pose.X(), current_pose.Y()}
        : waypoints.back();
    double edx = units::meter_t{lastPoint[0]}.value() - lastWaypoint.X().value();
    double edy = units::meter_t{lastPoint[1]}.value() - lastWaypoint.Y().value();
    frc::Rotation2d endHeading{units::radian_t{std::atan2(edy, edx)}};

    frc::Pose2d endPose{
        units::meter_t{lastPoint[0]},
        units::meter_t{lastPoint[1]},
        endHeading
    };

    // Build config and generate trajectory using the current max speed
    frc::TrajectoryConfig config(
        swerve_coordinator->getMaxSpeed(),
        units::meters_per_second_squared_t{maxAccel}
    );

    auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose, waypoints, endPose, config
    );

    std::cout << "followPathLua: startPose=(" << startPose.X().value() << ", " << startPose.Y().value() 
              << ", heading=" << startPose.Rotation().Degrees().value() << "°)" << std::endl;
    std::cout << "followPathLua: endPose=(" << endPose.X().value() << ", " << endPose.Y().value()
              << ", heading=" << endPose.Rotation().Degrees().value() << "°)" << std::endl;
    std::cout << "followPathLua: current_pose rotation=" << current_pose.Rotation().Degrees().value() << "°" << std::endl;
    std::cout << "followPathLua: endAngle (robot facing)=" << endAngle << "°" << std::endl;
    std::cout << "followPathLua: trajectory totalTime=" << trajectory.TotalTime().value() << "s, states=" << trajectory.States().size() << std::endl;

    // endAngle is the robot's desired facing direction at the end,
    // independent of the path travel direction.
    followTrajectory(trajectory, frc::Rotation2d{units::degree_t{endAngle}});
}

bool DriveSystem::isTrajectoryDoneLua()
{
    return isTrajectoryDone();
}

// ==== Private: Drive Mode Handlers ====

void DriveSystem::doStop()
{
    swerve_coordinator->stopDrive();
    swerve_coordinator->update();
}

void DriveSystem::doManualDrive()
{
    double stick_angle = HighVoltage::get180Range(HighVoltage::toDegrees(std::atan2(-manual_drive_y, manual_drive_x)) - 90.0);
    double stick_magnitude = std::sqrt(manual_drive_x*manual_drive_x + manual_drive_y*manual_drive_y);//std::pow is slow relative to multiplication, just multiply by itself to square
    stick_magnitude = std::min(1.0, stick_magnitude);
    units::degree_t current_angle = getRobotAngleInDegrees();
    if(stick_magnitude > DriveSystem::MANUAL_TRANSLATE_DEADZONE)
    {
        double turn = manual_rotate;
        if(std::abs(manual_rotate)>MANUAL_ROTATE_DEADZONE)
        {
            target_angle = current_angle;
            angle_correction_pid->Reset();
        }
        else
        {
            auto degrees_off = HighVoltage::get180Range(target_angle - current_angle);
            turn = angle_correction_pid->Calculate(degrees_off.value());
        }
        swerve_coordinator->setDrive(units::degree_t{stick_angle}, stick_magnitude, turn);
    }
    else if(std::abs(manual_rotate) > MANUAL_ROTATE_DEADZONE)
    {
        swerve_coordinator->setRotate(manual_rotate);
        target_angle = current_angle;
        angle_correction_pid->Reset();
    }
    else
    {
        swerve_coordinator->stopDrive();
    }
    swerve_coordinator->setFieldAngle(getRobotAngleInDegrees());
    swerve_coordinator->update();
}

void DriveSystem::doManualAA()
{
    double stick_angle = HighVoltage::get180Range(HighVoltage::toDegrees(std::atan2(-manual_drive_y, manual_drive_x)) - 90.0);
    double stick_magnitude = std::sqrt(manual_drive_x*manual_drive_x + manual_drive_y*manual_drive_y);//std::pow is slow relative to multiplication, just multiply by itself to square
    stick_magnitude = std::min(1.0, stick_magnitude);
    units::degree_t current_angle = getRobotAngleInDegrees();
    units::degree_t angle_error = angle_to_hub - getRobotAngleInDegrees() + auto_aim_offset;
    angle_error = HighVoltage::get180Range(angle_error);
    angle_error *= -1;
    // if(std::abs(angle_error.value()) > 90)
    // {
    //     angle_error -= 180_deg;
    //     angle_error = HighVoltage::get180Range(angle_error);
    // }

    if(stick_magnitude > DriveSystem::MANUAL_TRANSLATE_DEADZONE)
    {
        angle_correction_pid->SetSetpoint(0.0);
        double turn = -angle_correction_pid->Calculate(angle_error.value());
        swerve_coordinator->setDrive(units::degree_t{stick_angle}, stick_magnitude, turn);
    }
    else
    {
        if(std::abs(angle_error.value())> 0.25)
        {
            double power = angle_error.value() * rotate_to_proportional;
            swerve_coordinator->setRotate(power);
        }
        else
        {
            swerve_coordinator->stopDrive();
        }
    }
    target_angle = angle_to_hub;

    swerve_coordinator->update();
}

void DriveSystem::doManualFaceBack()
{
    double stick_angle = HighVoltage::get180Range(HighVoltage::toDegrees(std::atan2(-manual_drive_y, manual_drive_x)) - 90.0);
    double stick_magnitude = std::sqrt(manual_drive_x*manual_drive_x + manual_drive_y*manual_drive_y);
    stick_magnitude = std::min(1.0, stick_magnitude);
    units::degree_t current_angle = getRobotAngleInDegrees();
    units::degree_t angle_error = 180.0_deg - current_angle;
    angle_error = HighVoltage::get180Range(angle_error);
    angle_error *= -1;

    if(stick_magnitude > DriveSystem::MANUAL_TRANSLATE_DEADZONE)
    {
        angle_correction_pid->SetSetpoint(0.0);
        double turn = -angle_correction_pid->Calculate(angle_error.value());
        swerve_coordinator->setDrive(units::degree_t{stick_angle}, stick_magnitude, turn);
    }
    else
    {
        if(std::abs(angle_error.value()) > 0.25)
        {
            double power = angle_error.value() * rotate_to_proportional;
            swerve_coordinator->setRotate(power);
        }
        else
        {
            swerve_coordinator->stopDrive();
        }
    }
    target_angle = 180.0_deg;

    swerve_coordinator->update();
}

void DriveSystem::doDriveFF()
{
    front_left_module->pointTurnMotor(0_deg);
    front_right_module->pointTurnMotor(0_deg);
    back_left_module->pointTurnMotor(0_deg);
    back_right_module->pointTurnMotor(0_deg);
    front_left_module->updateDriveMotorVoltage(ff_voltage);
    front_right_module->updateDriveMotorVoltage(ff_voltage);
    back_left_module->updateDriveMotorVoltage(ff_voltage);
    back_right_module->updateDriveMotorVoltage(ff_voltage);
    std::vector<double> ff_data =
    {
        ff_voltage.value(), 
        front_left_module->getDriveMotor()->GetVelocity().GetValueAsDouble()*front_left_module->getDriveTurnsPerMeter(),
        front_right_module->getDriveMotor()->GetVelocity().GetValueAsDouble()*front_right_module->getDriveTurnsPerMeter(),
        back_left_module->getDriveMotor()->GetVelocity().GetValueAsDouble()*back_left_module->getDriveTurnsPerMeter(),
        back_right_module->getDriveMotor()->GetVelocity().GetValueAsDouble()*back_right_module->getDriveTurnsPerMeter()
    };
    nt_ff_data.setValue(ff_data);
}

void DriveSystem::doTurnFF()
{
    std::shared_ptr<HighVoltage::MK5NSwerveModule> target_module = nullptr;
    std::array<std::shared_ptr<HighVoltage::MK5NSwerveModule>, 4> all_modules = {
        front_left_module, front_right_module, back_left_module, back_right_module
    };
    switch(turn_ff_module_name)
    {
        case FRONT_LEFT:
            target_module = front_left_module;
            break;

        case FRONT_RIGHT:
            target_module = front_right_module;
            break;

        case BACK_LEFT:
            target_module = back_left_module;
            break;

        case BACK_RIGHT:
            target_module = back_right_module;
            break;
        
        default:
            target_module = front_left_module;
            break;
    }
    // Stop non-targeted modules
    for(auto& module : all_modules)
    {
        if(module != target_module)
        {
            module->updateDriveMotorVoltage(0_V);
            module->getRotateMotor()->SetVoltage(0_V);
        }
    }
    // Apply FF voltage only to the targeted module's rotate motor
    target_module->getRotateMotor()->SetVoltage(ff_voltage);
    std::vector<double> ff_data =
    {
        ff_voltage.value(), 
        target_module->getEncoder()->GetVelocity().GetValueAsDouble()*360.0
    };
    nt_ff_data.setValue(ff_data);
}

void DriveSystem::doDriveTo()
{
    auto angle = getAngleFromTarget();
    auto distance = getDistanceFromTarget();
    double speed = HighVoltage::clamp(min_auto_percentage, 1.0, drive_to_proportional * distance.value());
    units::degree_t current_angle = getRobotAngleInDegrees();
    units::degree_t angle_error = HighVoltage::get180Range(target_pose.Rotation().Degrees()-current_angle);
    double turn = angle_correction_pid->Calculate(angle_error.value(), 0);
    swerve_coordinator->setDrive(HighVoltage::get180Range(angle + 180_deg + current_angle), speed, turn);
    //swerve_cordinator->setFieldAngle(getRobotAngleInDegrees());
    swerve_coordinator->update();
}

void DriveSystem::doRotateTo()
{
    units::degree_t angle_error = target_angle - getRobotAngleInDegrees();
    angle_error = HighVoltage::get180Range(angle_error);
    angle_error *= -1;
    double power = angle_error.value() * rotate_to_proportional;
    swerve_coordinator->setRotate(power);
    swerve_coordinator->update();
}

void DriveSystem::doFollowTrajectory()
{
    auto elapsed = trajectory_timer.Get();

    // Check if the trajectory is complete
    if(elapsed > current_trajectory.TotalTime())
    {
        trajectory_done = true;
        trajectory_timer.Stop();
        swerve_coordinator->stopDrive();
        swerve_coordinator->update();
        drive_mode = STOP;
        return;
    }

    // Sample the desired state at the current time
    auto desired_state = current_trajectory.Sample(elapsed);

    // The robot's gyro convention is CW-positive, but WPILib's holonomic
    // controller and FromFieldRelativeSpeeds expect CCW-positive. Negate the
    // current pose rotation and the end rotation so the WPILib math works
    // correctly.
    frc::Pose2d wpilib_pose{current_pose.X(), current_pose.Y(),
                            frc::Rotation2d{-current_pose.Rotation().Degrees()}};
    frc::Rotation2d wpilib_end_rotation{-trajectory_end_rotation.Degrees()};

    // Calculate the chassis speeds needed to follow the trajectory.
    // The holonomic controller outputs field-relative ChassisSpeeds.
    auto field_speeds = holonomic_controller->Calculate(
        wpilib_pose,
        desired_state,
        wpilib_end_rotation
    );

    // Debug: print trajectory state every ~0.5s
    static int traj_debug_counter = 0;
    if(traj_debug_counter++ % 25 == 0)
    {
        std::cout << "[TRAJ t=" << elapsed.value() << "s] "
                  << "cur=(" << current_pose.X().value() << "," << current_pose.Y().value() 
                  << " rot=" << current_pose.Rotation().Degrees().value() << "°) "
                  << "des=(" << desired_state.pose.X().value() << "," << desired_state.pose.Y().value()
                  << " hdg=" << desired_state.pose.Rotation().Degrees().value() << "°) "
                  << "field_v=(" << field_speeds.vx.value() << "," << field_speeds.vy.value() 
                  << " omega=" << field_speeds.omega.value() << ")" << std::endl;
    }

    // Convert field-relative speeds to robot-relative speeds,
    // since ToSwerveModuleStates() expects robot-relative input.
    // Use the WPILib-convention (CCW-positive) rotation for the conversion.
    auto target_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        field_speeds.vx, field_speeds.vy, field_speeds.omega,
        wpilib_pose.Rotation()
    );

    // Use WPILib kinematics to convert ChassisSpeeds directly into
    // individual swerve module states. This avoids the angle convention
    // issues with the coordinator's setDrive() API.
    auto module_states = kinematics->ToSwerveModuleStates(target_speeds);
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
        &module_states, swerve_coordinator->getMaxSpeed());

    // Helper: apply a module state to a swerve module.
    // The MK5N module convention is 180° offset from WPILib convention
    // (module 0° = WPILib 180°), so wpilib_angle = module_angle + 180°,
    // meaning module_angle = wpilib_angle - 180°. We subtract 180° to
    // convert from WPILib to module convention, then let the module's
    // own update() handle optimization (forward/backward flip).
    auto applyModuleState = [](std::shared_ptr<HighVoltage::MK5NSwerveModule>& module,
                               const frc::SwerveModuleState& state)
    {
        // Shift angle by -180° to convert from WPILib convention to module convention
        module->setTargetAngle(state.angle.Degrees() - 180_deg);
        module->setSpeed(state.speed);
    };

    applyModuleState(front_left_module, module_states[0]);
    applyModuleState(front_right_module, module_states[1]);
    applyModuleState(back_left_module, module_states[2]);
    applyModuleState(back_right_module, module_states[3]);

    // Run each module's control loop (PID + feedforward for drive and turn motors).
    // Encoder offsets are applied inside each module's update().
    front_left_module->update(std::chrono::milliseconds{20});
    front_right_module->update(std::chrono::milliseconds{20});
    back_left_module->update(std::chrono::milliseconds{20});
    back_right_module->update(std::chrono::milliseconds{20});
}

// ==== Private: Internal Helpers ====

frc::Pose2d DriveSystem::updateOdometry()
{
    // Batch all 6 Pigeon signal reads into one CAN round-trip instead of 6 separate blocking calls
    auto& pigeon = context->getPigeon();
    auto sig_yaw       = pigeon.GetYaw();
    auto sig_yaw_rate  = pigeon.GetAngularVelocityXDevice();
    auto sig_pitch     = pigeon.GetPitch();
    auto sig_pitch_rate= pigeon.GetAngularVelocityYDevice();
    auto sig_roll      = pigeon.GetRoll();
    auto sig_roll_rate = pigeon.GetAngularVelocityZDevice();
    ctre::phoenix6::BaseStatusSignal::RefreshAll(sig_yaw, sig_yaw_rate, sig_pitch, sig_pitch_rate, sig_roll, sig_roll_rate);

    double yaw       = sig_yaw.GetValue().to<double>();
    double yaw_rate  = sig_yaw_rate.GetValue().to<double>();
    double pitch     = sig_pitch.GetValue().to<double>();
    double pitch_rate= sig_pitch_rate.GetValue().to<double>();
    double roll      = sig_roll.GetValue().to<double>();
    double roll_rate = sig_roll_rate.GetValue().to<double>();

    // Cache the robot angle from the batch-read yaw so getRobotAngleInDegrees()
    // does not trigger additional CAN reads for the rest of this iteration.
    cached_robot_angle_ = HighVoltage::get180Range(units::degree_t{yaw} - context->getStartAngle());

    //If we are on the red alliance we need to flip the yaw
    yaw -= context->getStartAngle().value();
    //If we are on the red alliance we need to flip the yaw
    auto current_alliance = frc::DriverStation::GetAlliance();
    if(!current_alliance.has_value())
    {
        return frc::Pose2d{};
    }
    if(current_alliance.value() == frc::DriverStation::kRed)
    {
        yaw = HighVoltage::get180Range(yaw + 180.0);
    }
    context->getVision().setRobotOrientation(
        yaw,
        yaw_rate,
        pitch,
        pitch_rate,
        roll,
        roll_rate
    );

    //Apply vision data to odometry
    context->getVision().applyVisionData(*odometry);
    return odometry->Update(
        getRobotAngleInDegrees(),
        std::array<frc::SwerveModulePosition,4>{
            front_left_module->getModulePosition(),
            front_right_module->getModulePosition(),
            back_left_module->getModulePosition(),
            back_right_module->getModulePosition(),
        }
    );
    
}

void DriveSystem::setNTThrottleInterval(int interval)
{
    drive_nt_throttle_interval_ = interval;
    front_left_module->setNTThrottleInterval(interval);
    front_right_module->setNTThrottleInterval(interval);
    back_left_module->setNTThrottleInterval(interval);
    back_right_module->setNTThrottleInterval(interval);
}

void DriveSystem::updateNetworkTables()
{
    // Throttle NT updates: 0 or 1 = every loop, N = every Nth loop
    if (drive_nt_throttle_interval_ > 1)
    {
        drive_nt_counter_++;
        if (drive_nt_counter_ < drive_nt_throttle_interval_) return;
        drive_nt_counter_ = 0;
    }

    nt_drive_mode.setValue(driveModeToString(drive_mode));
    nt_drive_x.setValue(current_pose.X().value());
    nt_drive_y.setValue(current_pose.Y().value());
    nt_drive_angle.setValue(current_pose.Rotation().Degrees().value());
    nt_target_drive_x.setValue(target_pose.X().value());
    nt_target_drive_y.setValue(target_pose.Y().value());
    nt_target_drive_angle.setValue(target_pose.Rotation().Degrees().value());
    nt_angle_to_hub.setValue(angle_to_hub.value());
    nt_distance_to_hub.setValue(distance_to_hub.value());
    nt_locked_on.setValue(isLockedOn());
}
