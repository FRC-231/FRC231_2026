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
    j.at(DriveConfig::ANGLE_CORRECTION_PID).get_to(drive.angle_correction_pid);
    j.at(DriveConfig::DRIVE_TO_PROPORTIONAL).get_to(drive.drive_to_proportional);
    

}

void from_json(const nlohmann::json &j, IntakeConfig &intake)
{
    j.at(IntakeConfig::PIVOT_MOTOR_ID).get_to(intake.pivot_motor_id);
    j.at(IntakeConfig::FEED_MOTOR_ID).get_to(intake.feed_motor_id);
    j.at(IntakeConfig::PIVOT_ENCODER_ID).get_to(intake.pivot_encoder_id);
    j.at(IntakeConfig::FEED_INTAKE_VOLTAGE).get_to(intake.feed_intake_voltage);
    j.at(IntakeConfig::FEED_OUTTAKE_VOLTAGE).get_to(intake.feed_outtake_voltage);
    j.at(IntakeConfig::PIVOT_MAX_VOLTAGE).get_to(intake.pivot_max_voltage);
    j.at(IntakeConfig::PIVOT_MIN_VOLTAGE).get_to(intake.pivot_min_voltage);
    j.at(IntakeConfig::PIVOT_MAX_ANGLE).get_to(intake.pivot_max_angle);
    j.at(IntakeConfig::PIVOT_MIN_ANGLE).get_to(intake.pivot_min_angle);

}

void from_json(const nlohmann::json &j, RobotConfig &robot)
{
    j.at(RobotConfig::DRIVE).get_to(robot.drive_config);
    j.at(RobotConfig::INTAKE).get_to(robot.intake_config);
}
}