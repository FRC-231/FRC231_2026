#include "Debounce.h"

namespace HighVoltage
{

Debounce::Debounce(std::chrono::microseconds delay)
{
    debounce_delay = delay;
}

void Debounce::setValue(bool input)
{
    std::lock_guard<std::mutex> lg (lock);
    if(input != last_input)
    {
        last_update = std::chrono::steady_clock::now();
        is_debouncing = true;
    }
    last_input = input;
}

bool Debounce::getValue()
{
    std::lock_guard<std::mutex> lg (lock);
    if(is_debouncing)
    {
        auto end_time = last_update + debounce_delay;
        if(std::chrono::steady_clock::now() >= end_time)
        {
            output = last_input;
            is_debouncing = false;
        }
    }
    return output;
}

void Debounce::forceValue(bool input)
{
    std::lock_guard<std::mutex> lg (lock);
    last_input = input;
    is_debouncing = false;
    last_update = std::chrono::steady_clock::now();
}

Debounce& Debounce::operator=(const bool& value)
{
    setValue(value);
    return *this;
}

Debounce::operator bool() const
{
    return output;
}

}
