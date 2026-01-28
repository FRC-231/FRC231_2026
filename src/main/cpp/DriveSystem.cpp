#include "DriveSystem.h"

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
}
void DriveSystem::Periodic()
{
    swerve_cordinator->update();
}


void DriveSystem::setDrive(double x, double y, double rotate)
{
    double stick_angle = HighVoltage::get180Range(HighVoltage::toDegrees(std::atan2(-y, x)) - 90.0);
    double stick_magnitude = std::sqrt(x*x + y*y);//std::pow is slow relative to multiplication, just multiply by itself to square
    stick_magnitude = std::min(1.0, stick_magnitude);
    if(stick_magnitude > DriveSystem::MANUAL_TRANSLATE_DEADZONE)
    {
        swerve_cordinator->setDrive(units::degree_t{stick_angle}, stick_magnitude, rotate);
    }
}