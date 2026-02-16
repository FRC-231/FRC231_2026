#include "NetworkTable/StringSelector.h"


namespace HighVoltage
{
namespace NetworkTable
{


HighVoltage::NetworkTable::StringSelector::StringSelector(const std::string& nt_prefix, const std::vector<std::string>& options, const std::string& default_option)
{
    if (std::find(options.begin(), options.end(), default_option) == options.end())
    {
        throw std::invalid_argument("Default option must be one of the options");
    }

    this->nt_prefix = nt_prefix;
    //If the prefix does not end with a slash add one
    if (!this->nt_prefix.empty() && this->nt_prefix.back() != '/')
    {
        this->nt_prefix += '/';
    }
    this->options = options;
    this->default_option = default_option;
    this->selected_option = default_option;

    nt_value = std::make_unique<NetworkTableValue<std::string>>(this->nt_prefix + SELECTED_KEY, default_option);
    nt_options = std::make_unique<NetworkTableValue<std::vector<std::string>>>(this->nt_prefix + OPTIONS_KEY, options);

    nt_value->setOnUpdate([this](const std::string& new_value)
    {
        setSelected(new_value, false);
        if(on_update_callback)
        {
            on_update_callback(new_value);
        }
    });
}


void HighVoltage::NetworkTable::StringSelector::setSelected(const std::string& option, bool update_nt)
{
    if (option == selected_option)
    {
        return;
    }

    if (std::find(options.begin(), options.end(), option) == options.end())
    {
        return;
    }

    selected_option = option;
    nt_value->setValue(option);
}

std::string HighVoltage::NetworkTable::StringSelector::getSelected()
{
    return selected_option;
}

std::vector<std::string> HighVoltage::NetworkTable::StringSelector::getOptions()
{
    return options;
}

void HighVoltage::NetworkTable::StringSelector::setOnUpdateCallback(std::function<void(const std::string&)> callback)
{
    on_update_callback = callback;
}


} // namespace NetworkTable
} // namespace HighVoltage

