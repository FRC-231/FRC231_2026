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


