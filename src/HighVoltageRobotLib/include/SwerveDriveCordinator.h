#pragma once

#include <memory>
#include <units/angle.h>
#include <units/velocity.h>

#include "SwerveModule.h"

namespace HighVoltage
{
class SwerveModuleCordinator
{
public:
    
    SwerveModuleCordinator() = delete;
    SwerveModuleCordinator(const SwerveModuleCordinator&) = delete;

    SwerveModuleCordinator(
        std::shared_ptr<SwerveModule> fl,
        std::shared_ptr<SwerveModule> fr,
        std::shared_ptr<SwerveModule> bl,
        std::shared_ptr<SwerveModule> br,
        double w_base,
        double t_width,
        units::meters_per_second_t max_speed = 1.0_mps
    );

    void setDrive(units::angle::degree_t angle, double magnitude, double turn);
    void setRotate(double turn);
    void stopDrive();
    void update(std::chrono::duration<double> seconds_passed = std::chrono::milliseconds{50});

    void setFieldOriented(bool is_field_oriented);
    bool isFieldOriented();
    void setFieldAngle(units::angle::degree_t angle);
    void setMaxSpeed(units::meters_per_second_t speed);
    units::meters_per_second_t getMaxSpeed();

private:

    std::shared_ptr<SwerveModule> front_left{nullptr};
    std::shared_ptr<SwerveModule> front_right{nullptr};
    std::shared_ptr<SwerveModule> back_left{nullptr};
    std::shared_ptr<SwerveModule> back_right{nullptr};

    double wheel_base;
    double track_width;

    units::angle::degree_t front_left_angle{0};
    units::angle::degree_t front_right_angle{0};
    units::angle::degree_t back_left_angle{0};
    units::angle::degree_t back_right_angle{0};

    bool field_oriented{true};
    units::angle::degree_t field_angle{0.0};
    units::meters_per_second_t max_speed{1.0_mps};

};
} // namespace HighVoltage

