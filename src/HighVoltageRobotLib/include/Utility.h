#pragma once

#include <vector>
#include <optional>
#include <frc/DriverStation.h>
#include <units/angle.h>

namespace HighVoltage
{

static const inline double PI = 3.141592653589793238463;

double get180Range(double angle);
units::degree_t get180Range(units::degree_t angle);
double get360Range(double angle);
units::degree_t get360Range(units::degree_t angle);
double closestAngle(double a, double b);
units::degree_t closestAngle(units::degree_t a, units::degree_t b);
double toDegrees(double radians);
double toRadians(double degrees);

//Clamps a provided value between a min and max value
template<typename T>
T clamp(T min, T max, T value)
{
    if(value < min)
    {
        return min;
    }
    else if(value > max)
    {
        return max;
    }
    return value;
}

//Returns 0 if the input is within the deadzone, otherwise returns the input
template<typename T>
T deadZone(T input, T deadzone)
{
    if(std::abs(input) <= deadzone)
    {
        return 0;
    }
    return input;
}

}

