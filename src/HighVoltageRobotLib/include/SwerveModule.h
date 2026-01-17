#pragma once

#include <chrono>

#include "Utility.h"

#include <frc/kinematics/SwerveModulePosition.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/length.h>
namespace HighVoltage
{

class SwerveModule
{
public:

    SwerveModule(std::string name): module_name{name} {};

    void setTargetAngle(units::angle::degree_t target);
    void setSpeed(units::velocity::meters_per_second_t speed);
    units::angle::degree_t getTargetAngle();
    units::velocity::meters_per_second_t getSpeed();
    virtual void update(std::chrono::duration<double> seconds_passed) = 0;
    virtual void resetError();
    virtual frc::SwerveModulePosition getModulePosition() = 0;
    virtual units::angle::degree_t getRawAngle() = 0;
    virtual units::angle::degree_t getAngle() = 0;
    virtual units::length::meter_t getEncoderPosition() = 0;

protected:
    units::angle::degree_t target_angle{0};
    units::velocity::meters_per_second_t target_speed{0};
    std::string module_name;

};

}
