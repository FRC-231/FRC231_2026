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

    nt_drive_mode.setOnUpdate([this](const std::string& mode_str)
    {
        this->drive_mode = stringToDriveMode(mode_str);
    });

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

void DriveSystem::setTurnFF(units::volt_t voltage)
{
    drive_mode = DriveMode::TURN_FF;
    ff_voltage = voltage;
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
        front_left_module->getDriveMotor()->GetVelocity().GetValueAsDouble(),
        front_right_module->getDriveMotor()->GetVelocity().GetValueAsDouble(),
        back_left_module->getDriveMotor()->GetVelocity().GetValueAsDouble(),
        back_right_module->getDriveMotor()->GetVelocity().GetValueAsDouble()
    };
    nt_ff_data.setValue(ff_data);
}

void DriveSystem::doTurnFF()
{
    front_left_module->getRotateMotor()->SetVoltage(ff_voltage);
    front_right_module->getRotateMotor()->SetVoltage(ff_voltage);
    back_left_module->getRotateMotor()->SetVoltage(ff_voltage);
    back_right_module->getRotateMotor()->SetVoltage(ff_voltage);
    std::vector<double> ff_data =
    {
        ff_voltage.value(), 
        front_left_module->getEncoder()->GetVelocity().GetValueAsDouble()*360.0,
        front_right_module->getEncoder()->GetVelocity().GetValueAsDouble()*360.0,
        back_left_module->getEncoder()->GetVelocity().GetValueAsDouble()*360.0,
        back_right_module->getEncoder()->GetVelocity().GetValueAsDouble()*360.0
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

void DriveSystem::stopDrive()
{
    drive_mode = STOP;
}

void DriveSystem::updateNetworkTables()
{
    nt_drive_mode.setValue(driveModeToString(drive_mode));
}

