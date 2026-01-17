#include "SwerveModule.h"

namespace HighVoltage
{

void SwerveModule::setTargetAngle(units::angle::degree_t target)
{
    target_angle = target;
}

units::angle::degree_t SwerveModule::getTargetAngle()
{
    return target_angle;
}

void SwerveModule::setSpeed(units::velocity::meters_per_second_t speed)
{
    target_speed = speed;
}

units::velocity::meters_per_second_t SwerveModule::getSpeed()
{
    return target_speed;
}

void SwerveModule::resetError()
{
    // Default implementation does nothing
}

}
