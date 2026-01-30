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

    swerve_cordinator = std::make_unique<HighVoltage::SwerveModuleCordinator>(
        front_left_module,
        front_right_module,
        back_left_module,
        back_right_module,
        0.6096, //TODO: Use config values
        0.6096 //TODO: Use config values
    );
    swerve_cordinator->setMaxSpeed(4_mps);
    swerve_cordinator->setFieldOriented(true);
}
void DriveSystem::Periodic()
{
    switch(drive_mode)
    {
        case STOP:
        {
            swerve_cordinator->stopDrive();
        }break;

        case MANUAL:
        {
            doManualDrive();
        }break;

        default:
        {
            swerve_cordinator->stopDrive();
        }break;
    }
    swerve_cordinator->setFieldAngle(getRobotAngleInDegrees());
    swerve_cordinator->update();
}


void DriveSystem::setDrive(double x, double y, double rotate)
{
    drive_mode = DriveMode::MANUAL;
    manual_drive_x = x;
    manual_drive_y = y;
    manual_rotate = rotate;
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
    if(stick_magnitude > DriveSystem::MANUAL_TRANSLATE_DEADZONE)
    {
        swerve_cordinator->setDrive(units::degree_t{stick_angle}, stick_magnitude, manual_rotate);
    }
    else if(std::abs(manual_rotate) > MANUAL_ROTATE_DEADZONE)
    {
        swerve_cordinator->setRotate(manual_rotate);
    }
    else
    {
        swerve_cordinator->stopDrive();
    }
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

void DriveSystem::stopDrive()
{
    drive_mode = STOP;
}

