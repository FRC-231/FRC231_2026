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

class StringSelector
{
public:
    StringSelector(const std::string& nt_prefix, const std::vector<std::string>& options, const std::string& default_option);
    StringSelector() = delete; // Disable default constructor

    static const inline std::string SELECTED_KEY = "Selected";
    static const inline std::string OPTIONS_KEY = "Options";

    void setSelected(const std::string& option, bool update_nt = true);
    std::string getSelected();
    std::vector<std::string> getOptions();
    void setOnUpdateCallback(std::function<void(const std::string&)> callback);

private:
    std::string nt_prefix;
    std::vector<std::string> options;
    std::string default_option;
    std::string selected_option;
    std::function<void(const std::string&)> on_update_callback;

    std::unique_ptr<NetworkTableValue<std::string>> nt_value;
    std::unique_ptr<NetworkTableValue<std::vector<std::string>>> nt_options;
};

} // namespace NetworkTable
} // namespace HighVoltage