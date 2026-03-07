#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <memory>

#include "NetworkTableValue.h"


namespace HighVoltage
{
namespace NetworkTable
{

class Button
{
public:
    Button(const std::string& nt_key);
    Button() = delete; // Disable default constructor

    bool getValue();
    bool wasPressed();
    bool wasReleased();

private:
    std::string nt_key;
    std::unique_ptr<NetworkTableValue<bool>> nt_value;
    bool last_value{false};
};


}// namespace NetworkTable
} // namespace HighVoltage