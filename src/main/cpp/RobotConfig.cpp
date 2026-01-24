#include "RobotConfig.h"
namespace RobotConfig
{
void from_json(const nlohmann::json &j, DriveConfig &drive)
{
    j.at(DriveConfig::SWERVE_KINEMATICS).get_to(drive.swerve_kinematics);
    j.at(DriveConfig::ROTATE_FEED_FORWARD).get_to(drive.rotate_feed_forward);
    j.at(DriveConfig::FRONT_LEFT_MODULE).get_to(drive.front_left_module);
    j.at(DriveConfig::FRONT_RIGHT_MODULE).get_to(drive.front_right_module);
    j.at(DriveConfig::BACK_LEFT_MODULE).get_to(drive.back_left_module);
    j.at(DriveConfig::BACK_RIGHT_MODULE).get_to(drive.back_right_module);
}
void from_json(const nlohmann::json &j, RobotConfig &robot)
{
    j.at(RobotConfig::DRIVE).get_to(robot.drive_config);
}
void from_json(const nlohmann::json &j, DriveConfig::SwerveModuleInfo &module)
{
    j.at(DriveConfig::SwerveModuleInfo::ROTATE_ID).get_to(module.rotate_id);
    j.at(DriveConfig::SwerveModuleInfo::DRIVE_ID).get_to(module.drive_id);
    j.at(DriveConfig::SwerveModuleInfo::ENCODER_ID).get_to(module.encoder_id);
    j.at(DriveConfig::SwerveModuleInfo::ENCODER_OFFSET).get_to(module.encoder_offset);
}
}