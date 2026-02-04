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
    swerve_cordinator->setMaxSpeed(1.5_mps);
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
void DriveSystem::Periodic()
{
    current_pose = updateOdometry();
    swerve_cordinator->setFieldAngle(getRobotAngleInDegrees());
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

        case DRIVE_FF:
        {
            doDriveFF();
        }break;

        case TURN_FF:
        {
            doTurnFF();
        }break;

        default:
        {
            doStop();
        }break;
    }
    updateNetworkTables();
}


void DriveSystem::setDrive(double x, double y, double rotate)
{
    drive_mode = DriveMode::MANUAL;
    manual_drive_x = x;
    manual_drive_y = y;
    manual_rotate = rotate;
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

void DriveSystem::doStop()
{
    swerve_cordinator->stopDrive();
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

units::degree_t DriveSystem::getRobotAngleInDegrees()
{
    return HighVoltage::get180Range(context->getPigeon().GetYaw().GetValue() - context->getStartAngle()+180_deg);
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

void DriveSystem::stopDrive()
{
    drive_mode = STOP;
}

void DriveSystem::updateNetworkTables()
{
    nt_drive_mode.setValue(driveModeToString(drive_mode));
    nt_drive_x.setValue(current_pose.X().value());
    nt_drive_y.setValue(current_pose.Y().value());
    nt_drive_angle.setValue(current_pose.Rotation().Degrees().value());

}

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
        std::cerr << "No alliance color set!" << std::endl;
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
