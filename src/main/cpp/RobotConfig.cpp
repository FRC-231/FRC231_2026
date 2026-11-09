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
    j.at(DriveConfig::AUTO_AIM_OFFSET).get_to(drive.auto_aim_offset);
    

}

void from_json(const nlohmann::json &j, IntakeConfig &intake)
{
    j.at(IntakeConfig::PIVOT_MOTOR_ID).get_to(intake.pivot_motor_id);
    j.at(IntakeConfig::FEED_MOTOR_ID).get_to(intake.feed_motor_id);
    j.at(IntakeConfig::FEED_MOTOR2_ID).get_to(intake.feed_motor2_id);
    j.at(IntakeConfig::PIVOT_ENCODER_ID).get_to(intake.pivot_encoder_id);
    j.at(IntakeConfig::FEED_INTAKE_VOLTAGE).get_to(intake.feed_intake_voltage);
    j.at(IntakeConfig::FEED_OUTTAKE_VOLTAGE).get_to(intake.feed_outtake_voltage);
    j.at(IntakeConfig::PIVOT_MAX_VOLTAGE).get_to(intake.pivot_max_voltage);
    j.at(IntakeConfig::PIVOT_MIN_VOLTAGE).get_to(intake.pivot_min_voltage);
    j.at(IntakeConfig::PIVOT_MAX_ANGLE).get_to(intake.pivot_max_angle);
    j.at(IntakeConfig::PIVOT_MIN_ANGLE).get_to(intake.pivot_min_angle);
    j.at(IntakeConfig::PIVOT_PID).get_to(intake.pivot_pid);
    j.at(IntakeConfig::PIVOT_DOWN_ANGLE).get_to(intake.pivot_down_angle);
    j.at(IntakeConfig::PIVOT_UP_ANGLE).get_to(intake.pivot_up_angle);
    j.at(IntakeConfig::PIVOT_WIGGLE_UP_ANGLE).get_to(intake.pivot_wiggle_up_angle);
    j.at(IntakeConfig::PIVOT_WIGGLE_DOWN_ANGLE).get_to(intake.pivot_wiggle_down_angle);
    j.at(IntakeConfig::PIVOT_WIGGLE_PERIOD).get_to(intake.pivot_wiggle_period);
    j.at(IntakeConfig::PIVOT_ENCODER_OFFSET).get_to(intake.pivot_encoder_offset);
    j.at(IntakeConfig::DANGER_HEIGHT).get_to(intake.danger_height);

}

void from_json(const nlohmann::json &j, ShooterConfig &shooter)
{
    j.at(ShooterConfig::HOPPER_MOTOR_ID).get_to(shooter.hopper_motor_id);
    j.at(ShooterConfig::HOPPER_MOTOR2_ID).get_to(shooter.hopper_motor2_id);
    j.at(ShooterConfig::FEED_MOTOR_ID).get_to(shooter.feed_motor_id);
    j.at(ShooterConfig::FEED_MOTOR2_ID).get_to(shooter.feed_motor2_id);
    j.at(ShooterConfig::LEFT_SHOOTER_MOTOR_ID).get_to(shooter.left_shooter_motor_id);
    j.at(ShooterConfig::RIGHT_SHOOTER_MOTOR_ID).get_to(shooter.right_shooter_motor_id);
    j.at(ShooterConfig::LEFT_SHOOTER_MOTOR2_ID).get_to(shooter.left_shooter_motor2_id);
    j.at(ShooterConfig::RIGHT_SHOOTER_MOTOR2_ID).get_to(shooter.right_shooter_motor2_id);
    j.at(ShooterConfig::SERVO_CONTROLLER_ID).get_to(shooter.servo_controller_id);
    j.at(ShooterConfig::LEFT_SERVO_ID).get_to(shooter.left_servo_id);
    j.at(ShooterConfig::RIGHT_SERVO_ID).get_to(shooter.right_servo_id);
    j.at(ShooterConfig::SHOOTER_PID).get_to(shooter.shooter_pid);
    j.at(ShooterConfig::HOPPER_VOLTAGE).get_to(shooter.hopper_voltage);
    j.at(ShooterConfig::HOPPER_WIGGLE_LOW_VOLTAGE).get_to(shooter.hopper_wiggle_low_voltage);
    j.at(ShooterConfig::HOPPER_WIGGLE_PERIOD).get_to(shooter.hopper_wiggle_period);
    j.at(ShooterConfig::FEED_VOLTAGE).get_to(shooter.feed_voltage);
    j.at(ShooterConfig::SHOOTER_FF).get_to(shooter.shooter_ff);
    j.at(ShooterConfig::SHOOTER_GEAR_RATIO).get_to(shooter.shooter_gear_ratio);
    j.at(ShooterConfig::SHOOTER_MAX_VOLTAGE).get_to(shooter.shooter_max_voltage);
    j.at(ShooterConfig::SHOOTER_MIN_VOLTAGE).get_to(shooter.shooter_min_voltage);
    j.at(ShooterConfig::AT_SPEED_TOLERANCE).get_to(shooter.at_speed_tolerance);
    j.at(ShooterConfig::IDLE_MODE_VELOCITY).get_to(shooter.idle_mode_velocity);
    j.at(ShooterConfig::SNOW_BLOW_TARGET_RPM).get_to(shooter.snow_blow_target_rpm);
    j.at(ShooterConfig::SNOW_BLOW_PULSE_WIDTH).get_to(shooter.snow_blow_pulse_width);
    j.at(ShooterConfig::SNOW_BLOW_PROPORTIONAL).get_to(shooter.snow_blow_proportional);
    j.at(ShooterConfig::MANUAL_SHOOTER_ADJUSTMENT_MULTIPLIER).get_to(shooter.manual_shooter_adjustment_multiplier);
    // Optional — defaults to 0 (no ramp) if absent
    if (j.contains(ShooterConfig::SHOOTER_RAMP_ACCELERATION))
    {
        j.at(ShooterConfig::SHOOTER_RAMP_ACCELERATION).get_to(shooter.shooter_ramp_acceleration);
    }
    //Load the distance to shooter velocity map. Will be an array of arrays, where the inner arrays have 2 elements: distance and velocity
    std::vector<std::vector<double>> distance_to_velocity_vec;
    j.at(ShooterConfig::DISTANCE_TO_SHOOTER_VELOCITY).get_to(distance_to_velocity_vec);
    for(const auto& pair : distance_to_velocity_vec)
    {    
        if(pair.size() != 2)
        {
            throw std::runtime_error("Invalid distance to shooter velocity pair in config");
        }
        shooter.distance_to_shooter_velocity[pair[0]] = pair[1];
    }
    //Load the distance to pulse width map
    std::vector<std::vector<double>> distance_to_pulsewidth_vec;
    j.at(ShooterConfig::DISTANCE_TO_PULSEWIDTH).get_to(distance_to_pulsewidth_vec);
    for(const auto& pair : distance_to_pulsewidth_vec)
    {
        if(pair.size() != 2)
        {
            throw std::runtime_error("Invalid distance to pulse width pair in config");
        }
        shooter.distance_to_pulsewidth[pair[0]] = pair[1];
    }
}

void from_json(const nlohmann::json &j, RobotConfig &robot)
{
    j.at(RobotConfig::DRIVE).get_to(robot.drive_config);
    j.at(RobotConfig::INTAKE).get_to(robot.intake_config);
    j.at(RobotConfig::SHOOTER).get_to(robot.shooter_config);
    // Optional — defaults to false if missing from JSON
    if (j.contains(RobotConfig::COMPETITION_MODE))
    {
        j.at(RobotConfig::COMPETITION_MODE).get_to(robot.competition_mode);
    }
    // Optional — defaults to 5 if missing from JSON
    if (j.contains(RobotConfig::NT_THROTTLE_INTERVAL))
    {
        j.at(RobotConfig::NT_THROTTLE_INTERVAL).get_to(robot.nt_throttle_interval);
    }
}
}