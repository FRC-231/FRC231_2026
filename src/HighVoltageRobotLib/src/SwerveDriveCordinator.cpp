#include "SwerveDriveCordinator.h"

#include <cmath>
#include <iostream>
namespace HighVoltage
{
SwerveModuleCordinator::SwerveModuleCordinator(
    std::shared_ptr<SwerveModule> fl,
    std::shared_ptr<SwerveModule> fr,
    std::shared_ptr<SwerveModule> bl,
    std::shared_ptr<SwerveModule> br,
    double w_base,
    double t_width,
    units::meters_per_second_t max_speed
):
front_left{fl},
front_right{fr},
back_left{bl},
back_right{br},
wheel_base{w_base},
track_width{t_width},
max_speed{max_speed}
{
    //Calculate wheel angles from y+
    front_left_angle = units::degree_t{toDegrees(std::atan2(track_width, wheel_base))};
    front_right_angle = units::degree_t{toDegrees(std::atan2(-track_width, wheel_base))};
    back_left_angle = units::degree_t{toDegrees(std::atan2(track_width, -wheel_base))};
    back_right_angle = units::degree_t{toDegrees(std::atan2(-track_width, -wheel_base))};
}

void SwerveModuleCordinator::setDrive(units::degree_t angle, double magnitude, double turn)
{

    if(field_oriented)
    {
        angle = get180Range(angle - field_angle);
    }

    if(
        !front_left ||
        !front_right ||
        !back_left ||
        !back_right
    )
    {
        std::cout << "Not all motor pointers are set in the swerve cordinator!" << std::endl;
        return;
    }
    units::meters_per_second_t speed = max_speed*magnitude;
    front_left->setSpeed(speed);
    front_right->setSpeed(speed);
    back_left->setSpeed(speed);
    back_right->setSpeed(speed);
    turn = std::min(1.0, std::max(-1.0, turn));
    units::degree_t turn_angle = units::degree_t{turn*45.0};
    //Calculate the angle of the wheel for turning
    if(std::abs(closestAngle(angle, front_left_angle).value()) >= 90.0)
    {
        front_left->setTargetAngle(angle + turn_angle);
    }
    else
    {
        front_left->setTargetAngle(angle - turn_angle);
    }
    if(std::abs(closestAngle(angle, front_right_angle).value()) >= 90.0)
    {
        front_right->setTargetAngle(angle + turn_angle);
    }
    else
    {
        front_right->setTargetAngle(angle - turn_angle);
    }
    if(std::abs(closestAngle(angle, back_left_angle).value()) >= 90.0)
    {
        back_left->setTargetAngle(angle + turn_angle);
    }
    else
    {
        back_left->setTargetAngle(angle - turn_angle);
    }
    if(std::abs(closestAngle(angle, back_right_angle).value()) >= 90.0)
    {
        back_right->setTargetAngle(angle + turn_angle);
    }
    else
    {
        back_right->setTargetAngle(angle - turn_angle);
    }
}

void SwerveModuleCordinator::setRotate(double turn)
{
    if(
        !front_left ||
        !front_right ||
        !back_left ||
        !back_right
    )
    {
        std::cout << "Not all motor pointers are set in the swerve cordinator!" << std::endl;
        return;
    }
    units::meters_per_second_t turn_speed = max_speed*turn;
    front_left->setSpeed(turn_speed);
    front_right->setSpeed(turn_speed);
    back_left->setSpeed(turn_speed);
    back_right->setSpeed(turn_speed);

    front_left->setTargetAngle(front_left_angle - 90_deg);
    front_right->setTargetAngle(front_right_angle - 90_deg);
    back_left->setTargetAngle(back_left_angle - 90_deg);
    back_right->setTargetAngle(back_right_angle - 90_deg);
}

void SwerveModuleCordinator::stopDrive()
{
    front_left->setSpeed(0.0_mps);
    front_right->setSpeed(0.0_mps);
    back_left->setSpeed(0.0_mps);
    back_right->setSpeed(0.0_mps);
}

void SwerveModuleCordinator::update(std::chrono::duration<double> seconds_passed)
{
    front_left->update(seconds_passed);
    front_right->update(seconds_passed);
    back_left->update(seconds_passed);
    back_right->update(seconds_passed);
}

void SwerveModuleCordinator::setFieldOriented(bool is_field_oriented)
{
    field_oriented = is_field_oriented;
}

bool SwerveModuleCordinator::isFieldOriented()
{
    return field_oriented;
}

void SwerveModuleCordinator::setFieldAngle(units::angle::degree_t angle)
{
    field_angle = angle;
}

void SwerveModuleCordinator::setMaxSpeed(units::meters_per_second_t speed)
{
    max_speed = speed;
}

units::meters_per_second_t SwerveModuleCordinator::getMaxSpeed()
{
    return max_speed;
}

}//namespace HighVoltage
