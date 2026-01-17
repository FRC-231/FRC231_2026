#pragma once

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>
#include <frc/Timer.h>
#include <string_view>



namespace HighVoltage
{

struct PlayAudioMessage
{
    enum class EventType
    {
        PLAY_SOUND,
        STOP_ALL_SOUNDS,
        LOOP_SOUND,
        STOP_SOUND
    };

    static std::string eventTypeToString(EventType type)
    {
        switch(type)
        {
            case EventType::PLAY_SOUND: return "PLAY_SOUND";
            case EventType::STOP_ALL_SOUNDS: return "STOP_ALL_SOUNDS";
            case EventType::LOOP_SOUND: return "LOOP_SOUND";
            case EventType::STOP_SOUND: return "STOP_SOUND";
        }
        return "INVALID";
    }

    EventType event_type;
    std::optional<std::string> sound_name;

    static constexpr std::string_view EVENT_TYPE_KEY = "event_type";
    static constexpr std::string_view SOUND_NAME_KEY = "sound_name";
    static constexpr std::string_view TIME_KEY = "time";

    friend void to_json(nlohmann::json &j, const PlayAudioMessage &msg);
};

class AudioPlayer
{
public:
    AudioPlayer();
    ~AudioPlayer(){};

    void playSound(const std::string& sound_name);
    void stopAllSounds();
    void loopSound(const std::string& sound_name);
    void stopSound(const std::string& sound_name);

    static inline const PlayAudioMessage DEFAULT_MESSAGE = {PlayAudioMessage::EventType::STOP_ALL_SOUNDS, std::nullopt};
    static inline const std::string AUDIO_TABLE_NAME = "audio_player";
    static inline const std::string AUDIO_TOPIC_NAME = "audio_message";

private:
    nt::NetworkTableInstance nt_instance;
    std::shared_ptr<nt::NetworkTable> audio_table;
    nt::StringTopic audio_message_topic;
    nt::StringPublisher audio_message_publisher;


};
} // namespace HighVoltage
