#include "RobotConfig.h"
namespace RobotConfig
{
void from_json(const nlohmann::json &j, DriveConfig &drive)
{
    j.at(DriveConfig::SWERVE_KINEMATICS).get_to(drive.swerve_kinematics);
    j.at(DriveConfig::ROTATE_FEED_FORWARD).get_to(drive.rotate_feed_forward);
}
void from_json(const nlohmann::json &j, RobotConfig &robot)
{
    j.at(RobotConfig::DRIVE).get_to(robot.drive_config);
}
}