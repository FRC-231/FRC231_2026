#pragma once

#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/NetworkTableListener.h>
#include <optional>
#include <type_traits>
#include <string>
#include <vector>
#include <functional>

namespace HighVoltage
{

template <typename T>
class NetworkTableValue
{
public:

    NetworkTableValue(std::string key);
    NetworkTableValue(std::string key, T initial_value);

    void setValue(T value);
    T getValue(T default_value);
    void setOnUpdate(std::function<void(const T&)> callback);

private:
    using TopicType =
    std::conditional_t<std::is_same_v<T, bool>,
        nt::BooleanTopic,
    std::conditional_t<std::is_arithmetic_v<T> && !std::is_same_v<T, bool>,
        nt::DoubleTopic,
    std::conditional_t<std::is_same_v<T, std::string>,
        nt::StringTopic,
    std::conditional_t<std::is_same_v<T, std::vector<double>>,
        nt::DoubleArrayTopic,
    std::conditional_t<std::is_same_v<T, std::vector<std::string>>,
        nt::StringArrayTopic,
    void>>>>>;

    using PublisherType =
    std::conditional_t<std::is_same_v<T, bool>,
        nt::BooleanPublisher,
    std::conditional_t<std::is_arithmetic_v<T> && !std::is_same_v<T, bool>,
        nt::DoublePublisher,
    std::conditional_t<std::is_same_v<T, std::string>,
        nt::StringPublisher,
    std::conditional_t<std::is_same_v<T, std::vector<double>>,
        nt::DoubleArrayPublisher,
    std::conditional_t<std::is_same_v<T, std::vector<std::string>>,
        nt::StringArrayPublisher,
    void>>>>>;

    using SubscriberType =
    std::conditional_t<std::is_same_v<T, bool>,
        nt::BooleanSubscriber,
    std::conditional_t<std::is_arithmetic_v<T> && !std::is_same_v<T, bool>,
        nt::DoubleSubscriber,
    std::conditional_t<std::is_same_v<T, std::string>,
        nt::StringSubscriber,
    std::conditional_t<std::is_same_v<T, std::vector<double>>,
        nt::DoubleArraySubscriber,
    std::conditional_t<std::is_same_v<T, std::vector<std::string>>,
        nt::StringArraySubscriber,
    void>>>>>;

    TopicType topic;
    PublisherType publisher;
    std::optional<SubscriberType> subscriber;
    std::optional<NT_Listener> listener;
    // Listener lambda to handle updates
    std::function<void(const T&)> onUpdate;

};

// Helper functions to get the correct typed topic
namespace detail {
    template<typename T>
    auto GetTypedTopic(std::shared_ptr<nt::NetworkTable> table, const std::string& key) {
        if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
            return table->GetDoubleTopic(key);
        } else if constexpr (std::is_same_v<T, std::string>) {
            return table->GetStringTopic(key);
        } else if constexpr (std::is_same_v<T, bool>) {
            return table->GetBooleanTopic(key);
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            return table->GetDoubleArrayTopic(key);
        } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
            return table->GetStringArrayTopic(key);
        } else {
            static_assert(sizeof(T) == 0, "Unsupported type for NetworkTableValue");
        }
    }
}

// Template implementation must be in header file
// Default constructor
template <typename T>
NetworkTableValue<T>::NetworkTableValue(std::string key)
{
    auto network_table_inst = nt::NetworkTableInstance::GetDefault();
    auto table = network_table_inst.GetTable("");
    topic = detail::GetTypedTopic<T>(table, key);
    publisher = topic.Publish();
}

// Constructor with initial value
template <typename T>
NetworkTableValue<T>::NetworkTableValue(std::string key, T initial_value)
{
    auto network_table_inst = nt::NetworkTableInstance::GetDefault();
    auto table = network_table_inst.GetTable("");
    topic = detail::GetTypedTopic<T>(table, key);
    publisher = topic.Publish();
    publisher.SetDefault(initial_value);
    subscriber = topic.Subscribe(initial_value);
    listener = network_table_inst.AddListener(
        subscriber.value(),
        nt::EventFlags::kValueAll,
        [this](const nt::Event& event) {
            if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>)
            {
                if(event.GetValueEventData()->value.IsDouble())
                {
                    T new_value = static_cast<T>(event.GetValueEventData()->value.GetDouble());
                    if(onUpdate)
                    {
                        onUpdate(new_value);
                    }
                }
            }
            else if constexpr (std::is_same_v<T, std::string>)
            {
                if(event.GetValueEventData()->value.IsString())
                {
                    T new_value = std::string(event.GetValueEventData()->value.GetString());
                    if(onUpdate)
                    {
                        onUpdate(new_value);
                    }
                }
            }
            else if constexpr (std::is_same_v<T, bool>)
            {
                if(event.GetValueEventData()->value.IsBoolean())
                {
                    T new_value = event.GetValueEventData()->value.GetBoolean();
                    if(onUpdate)
                    {
                        onUpdate(new_value);
                    }
                }
            }
            else if constexpr (std::is_same_v<T, std::vector<double>>)
            {
                if(event.GetValueEventData()->value.IsDoubleArray())
                {
                    auto array = event.GetValueEventData()->value.GetDoubleArray();
                    T new_value(array.begin(), array.end());
                    if(onUpdate)
                    {
                        onUpdate(new_value);
                    }
                }
            }
            else if constexpr (std::is_same_v<T, std::vector<std::string>>)
            {
                if(event.GetValueEventData()->value.IsStringArray())
                {
                    auto array = event.GetValueEventData()->value.GetStringArray();
                    T new_value;
                    new_value.reserve(array.size());
                    for (const auto& str : array) {
                        new_value.push_back(std::string(str));
                    }
                    if(onUpdate)
                    {
                        onUpdate(new_value);
                    }
                }
            }
        }
    );
}

template <typename T>
void NetworkTableValue<T>::setValue(T value)
{
    if(!subscriber.has_value())
    {
        subscriber = topic.Subscribe(value);
    }
    publisher.Set(value);
}

template <typename T>
T NetworkTableValue<T>::getValue(T default_value)
{
    if (subscriber.has_value())
    {
        return subscriber->Get(default_value);
    }
    else
    {
        return default_value;
    }
}

template <typename T>
void NetworkTableValue<T>::setOnUpdate(std::function<void(const T&)> callback)
{
    onUpdate = callback;
}

} // namespace HighVoltage


