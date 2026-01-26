#include "RobotConfig.h"
namespace RobotConfig
{
void from_json(const nlohmann::json &j, DriveConfig &drive)
{
    j.at(DriveConfig::SWERVE_KINEMATICS).get_to(drive.swerve_kinematics);
    j.at(DriveConfig::FRONT_LEFT_MODULE).get_to(drive.front_left_module);
    j.at(DriveConfig::FRONT_RIGHT_MODULE).get_to(drive.front_right_module);
    j.at(DriveConfig::BACK_LEFT_MODULE).get_to(drive.back_left_module);
    j.at(DriveConfig::BACK_RIGHT_MODULE).get_to(drive.back_right_module);
}
void from_json(const nlohmann::json &j, RobotConfig &robot)
{
    j.at(RobotConfig::DRIVE).get_to(robot.drive_config);
}

}