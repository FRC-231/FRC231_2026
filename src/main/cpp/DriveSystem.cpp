#include "DriveSystem.h"
#include <iostream>

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

    swerve_cordinator = std::make_unique<HighVoltage::SwerveModuleCordinator>(
        front_left_module,
        front_right_module,
        back_left_module,
        back_right_module,
        0.6096, //TODO: Use config values
        0.6096 //TODO: Use config values
    );
    swerve_cordinator->setMaxSpeed(5.0_mps);
    swerve_cordinator->setFieldOriented(true);

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

}

// ==== Subsystem Override ====

void DriveSystem::Periodic()
{
    current_pose = updateOdometry();
    swerve_cordinator->setFieldAngle(getRobotAngleInDegrees());
    angle_to_hub = getAngleFromTargetPosition(RobotContext::HUB_POSITION);
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

void DriveSystem::setMaxSpeed(units::meters_per_second_t speed)
{
    swerve_cordinator->setMaxSpeed(speed);
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
    return HighVoltage::get180Range(context->getPigeon().GetYaw().GetValue() - context->getStartAngle());
}

units::degree_t DriveSystem::getRotationError()
{
    units::degree_t angle_error = target_angle - getRobotAngleInDegrees();
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

bool DriveSystem::isLockedOn()
{
    units::degree_t current_angle = getRobotAngleInDegrees();
    units::degree_t angle_error = angle_to_hub - current_angle;
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
    else if(mode_str == "DRIVE_TO_SHOOT_LOCATION")
    {
        return DRIVE_TO_SHOOT_LOCATION;
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

// ==== Private: Drive Mode Handlers ====

void DriveSystem::doStop()
{
    swerve_cordinator->stopDrive();
    swerve_cordinator->update();
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
            std::cout << degrees_off.value() << std::endl;
        }
        swerve_cordinator->setDrive(units::degree_t{stick_angle}, stick_magnitude, turn);
    }
    else if(std::abs(manual_rotate) > MANUAL_ROTATE_DEADZONE)
    {
        swerve_cordinator->setRotate(manual_rotate);
        target_angle = current_angle;
        angle_correction_pid->Reset();
    }
    else
    {
        swerve_cordinator->stopDrive();
    }
    swerve_cordinator->setFieldAngle(getRobotAngleInDegrees());
    swerve_cordinator->update();
}

void DriveSystem::doManualAA()
{
    double stick_angle = HighVoltage::get180Range(HighVoltage::toDegrees(std::atan2(-manual_drive_y, manual_drive_x)) - 90.0);
    double stick_magnitude = std::sqrt(manual_drive_x*manual_drive_x + manual_drive_y*manual_drive_y);//std::pow is slow relative to multiplication, just multiply by itself to square
    stick_magnitude = std::min(1.0, stick_magnitude);
    units::degree_t current_angle = getRobotAngleInDegrees();
    units::degree_t angle_error = angle_to_hub - getRobotAngleInDegrees();
    angle_error = HighVoltage::get180Range(angle_error);
    angle_error *= -1;
    std::cout << angle_error.value() << std::endl;
    if(std::abs(angle_error.value()) > 90)
    {
        angle_error -= 180_deg;
        angle_error = HighVoltage::get180Range(angle_error);
    }

    if(stick_magnitude > DriveSystem::MANUAL_TRANSLATE_DEADZONE)
    {
        angle_correction_pid->SetSetpoint(0.0);
        double turn = -angle_correction_pid->Calculate(angle_error.value());
        swerve_cordinator->setDrive(units::degree_t{stick_angle}, stick_magnitude, turn);
    }
    else
    {
        double power = angle_error.value() * rotate_to_proportional;
        swerve_cordinator->setRotate(power);
    }
    target_angle = angle_to_hub;
    swerve_cordinator->update();
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
    std::cout << angle.value() << " " << distance.value() << " " << current_angle.value() << std::endl;
    swerve_cordinator->setDrive(HighVoltage::get180Range(angle + 180_deg + current_angle), speed, turn);
    //swerve_cordinator->setFieldAngle(getRobotAngleInDegrees());
    swerve_cordinator->update();
}

void DriveSystem::doRotateTo()
{
    units::degree_t angle_error = target_angle - getRobotAngleInDegrees();
    angle_error *= -1;
    std::cout << angle_error.value() << std::endl;
    double power = angle_error.value() * rotate_to_proportional;
    swerve_cordinator->setRotate(power);
    swerve_cordinator->update();
}

// ==== Private: Internal Helpers ====

frc::Pose2d DriveSystem::updateOdometry()
{
    double yaw = context->getPigeon().GetYaw().GetValue().to<double>();
    double yaw_rate = context->getPigeon().GetAngularVelocityXDevice().GetValue().to<double>();
    double pitch = context->getPigeon().GetPitch().GetValue().to<double>();
    double pitch_rate = context->getPigeon().GetAngularVelocityYDevice().GetValue().to<double>();
    double roll = context->getPigeon().GetRoll().GetValue().to<double>();
    double roll_rate = context->getPigeon().GetAngularVelocityZDevice().GetValue().to<double>();
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

void DriveSystem::updateNetworkTables()
{
    nt_drive_mode.setValue(driveModeToString(drive_mode));
    nt_drive_x.setValue(current_pose.X().value());
    nt_drive_y.setValue(current_pose.Y().value());
    nt_drive_angle.setValue(current_pose.Rotation().Degrees().value());
    nt_target_drive_x.setValue(target_pose.X().value());
    nt_target_drive_y.setValue(target_pose.Y().value());
    nt_target_drive_angle.setValue(target_pose.Rotation().Degrees().value());
    nt_angle_to_hub.setValue(angle_to_hub.value());
    nt_locked_on.setValue(isLockedOn());
}
