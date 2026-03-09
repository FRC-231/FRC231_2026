#pragma once

#include <map>
#include <vector>
#include <optional>
#include <frc/DriverStation.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <units/velocity.h>
#include <units/acceleration.h>

namespace HighVoltage
{

// Type aliases for feedforward unit types
using VoltsPerMeterPerSecond = units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second>>>;
using VoltsPerMeterPerSecondSquared = units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second_squared>>>;

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

template<typename InputUnit, typename OutputUnit>
OutputUnit linearInterpolation(const std::map<InputUnit, OutputUnit>& data, InputUnit input) {
    if (data.empty()) return OutputUnit{0};

    // Clamp to range
    if (input <= data.begin()->first)  return data.begin()->second;
    if (input >= data.rbegin()->first) return data.rbegin()->second;

    // lower_bound gives us the first key >= input
    auto high = data.lower_bound(input);
    auto low  = std::prev(high);

    double t = (input - low->first).value() /
               (high->first - low->first).value();
    return low->second + t * (high->second - low->second);
}

}

