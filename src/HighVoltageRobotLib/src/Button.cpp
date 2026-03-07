#include "NetworkTable/Button.h"

namespace HighVoltage
{
namespace NetworkTable
{

Button::Button(const std::string& nt_key) : nt_key(nt_key)
{
    nt_value = std::make_unique<NetworkTableValue<bool>>(nt_key, false);
    last_value = nt_value->getValue(false);
}

bool Button::getValue()
{
    last_value = nt_value->getValue(false);
    return last_value;
}

bool Button::wasPressed()
{
    bool current_value = nt_value->getValue(false);
    bool was_pressed = !last_value && current_value;
    if(was_pressed)
    {
        last_value = current_value;
    }
    return was_pressed;
}

bool Button::wasReleased()
{
    bool current_value = nt_value->getValue(false);
    bool was_released = last_value && !current_value;
    if(was_released)
    {
        last_value = current_value;
    }
    return was_released;
}

} // namespace NetworkTable
} // namespace HighVoltage


