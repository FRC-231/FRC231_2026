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

//https://en.wikipedia.org/wiki/Linear_interpolation
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

//https://en.wikipedia.org/wiki/Cubic_Hermite_spline
template<typename InputUnit, typename OutputUnit>
OutputUnit cubicHermiteInterpolation(const std::map<InputUnit, OutputUnit>& data, InputUnit input)
{
    if (data.size() < 2) return linearInterpolation(data, input);

    // Clamp to range
    if (input <= data.begin()->first)  return data.begin()->second;
    if (input >= data.rbegin()->first) return data.rbegin()->second;

    // Find the segment: low <= input < high
    auto high = data.lower_bound(input);
    auto low  = std::prev(high);

    // Get p1 and p2 (the two points bracketing input)
    auto p1 = low;
    auto p2 = high;

    // Get p0 (point before p1, or p1 if at the start)
    auto p0 = (p1 != data.begin()) ? std::prev(p1) : p1;

    // Get p3 (point after p2, or p2 if at the end)
    auto p3 = std::next(p2);
    if (p3 == data.end()) p3 = p2;

    // Normalized parameter t in [0, 1] across the [p1, p2] segment
    double h = (p2->first - p1->first).value();
    double t = (input - p1->first).value() / h;

    // Catmull-Rom tangents (scaled to the segment width)
    double m1 = (p2->second - p0->second).value() /
                (p2->first  - p0->first).value() * h;
    double m2 = (p3->second - p1->second).value() /
                (p3->first  - p1->first).value() * h;

    // Hermite basis functions
    double t2 = t * t;
    double t3 = t2 * t;
    double h00 =  2*t3 - 3*t2 + 1;
    double h10 =    t3 - 2*t2 + t;
    double h01 = -2*t3 + 3*t2;
    double h11 =    t3 -   t2;

    // Evaluate the spline
    double result = h00 * p1->second.value()
                  + h10 * m1
                  + h01 * p2->second.value()
                  + h11 * m2;

    return OutputUnit{result};
}

} // namespace HighVoltage
