#pragma once

#include <chrono>
#include <atomic>
#include <mutex>

namespace HighVoltage
{

class Debounce
{
public:
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

    Debounce(std::chrono::microseconds delay);
    Debounce() = delete;
    
    Debounce& operator=(const bool& value);
    operator bool() const;

    void setValue(bool input);
    bool getValue();
    void forceValue(bool input);

private:
    TimePoint last_update{};
    bool output{false};
    bool last_input{false};
    std::chrono::microseconds debounce_delay;
    std::atomic<bool> is_debouncing{false};
    std::mutex lock;

};

}