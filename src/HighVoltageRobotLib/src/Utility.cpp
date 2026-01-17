#include "Utility.h"

#include <cmath>

namespace HighVoltage
{


double get180Range(double angle)
{
    angle = std::fmod(std::fmod(angle, 360) + 360, 360);
    if(angle > 180)
    {
        angle -= 360;
    }
    return angle;
}

units::degree_t get180Range(units::degree_t angle)
{
    return units::degree_t(get180Range(angle.value()));
}

double get360Range(double angle)
{
    angle = std::fmod(std::fmod(angle, 360) + 360, 360);
    return angle;
}

units::degree_t get360Range(units::degree_t angle)
{
    return units::degree_t(get360Range(angle.value()));
}

double closestAngle(double a, double b)
{
    double a_mod = std::fmod(a, 360);
    double b_mod = std::fmod(b, 360);
    return get180Range(b_mod - a_mod);
}

units::degree_t closestAngle(units::degree_t a, units::degree_t b)
{
    return units::degree_t(closestAngle(a.value(), b.value()));
}

double toDegrees(double radians)
{
    return radians * (180.0/PI);
}

units::degree_t toDegrees(units::radian_t radians)
{
    return units::degree_t(radians.value() * (180.0/PI));
}

double toRadians(double degrees)
{
    return degrees * (PI/180.0);
}

}

